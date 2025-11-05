/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 * (license text unchanged)
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// PedroPathing (Pinpoint) — used ONLY for heading; does not change driver commands.
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

@Config
@TeleOp(name = "NewTeleOp", group = "StarterBot")
//@Disabled
public class NewTeleOp extends OpMode {
    // ---- Driver feel (Dashboard tunables) ----
    public static double DEADBAND = 0.04;  // small stick deadzone
    public static double EXPO     = 0.60;  // 0 = linear, 1 = strong curve

    // ---- Shooter config ----
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // ---- Drive motors ----
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;

    // ---- Shooter hardware ----
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // ---- REV hubs (manual bulk caching) ----
    private List<LynxModule> hubs;

    ElapsedTime feederTimer = new ElapsedTime();

    // ---- Shooter state machine ----
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;

    // ---- Telemetry for mecanum ----
    double flPower, frPower, blPower, brPower;

    // ---- Pedro / Pinpoint (for IMU heading only) ----
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0); // set real pose if you want
    // Heading-hold PID (no new buttons)
    public static double kP_HOLD = 0.02;     // start small; tune on Dashboard
    public static double kD_HOLD = 0.001;    // light damping
    public static double ROT_DEADBAND = 0.03; // if |RS X| < this → hold heading
    private double headingTarget = 0.0;
    private double prevErr = 0.0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // Map hardware (names must match RC config exactly)
        FrontLeftDrive  = hardwareMap.get(DcMotor.class,  "fata_stanga");
        FrontRightDrive = hardwareMap.get(DcMotor.class,  "fata_dreapta");
        BackLeftDrive   = hardwareMap.get(DcMotor.class,  "spate_stanga");
        BackRightDrive  = hardwareMap.get(DcMotor.class,  "spate-dreapta"); // note the hyphen

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,   "right_feeder");

        // ---- Motor directions (baseline for mecanum) ----
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // ---- Braking ----
        FrontLeftDrive.setZeroPowerBehavior(BRAKE);
        FrontRightDrive.setZeroPowerBehavior(BRAKE);
        BackLeftDrive.setZeroPowerBehavior(BRAKE);
        BackRightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // ---- Launcher control mode & PIDF ----
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        // Feeders initial & direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Manual bulk caching (perf)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // ---- Pedro / Pinpoint setup (for heading only) ----
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        // do NOT send drive vectors to Pedro in TeleOp; we only call follower.update() to refresh pose

        dtTimer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // lock heading target to current Pinpoint heading at start
        follower.update();
        headingTarget = follower.getPose().getHeading(); // radians
        prevErr = 0.0;
        dtTimer.reset();
    }

    @Override
    public void loop() {
        // Clear caches once per loop (paired with MANUAL mode)
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // Update Pinpoint/Pedro pose (for heading)
        follower.update();
        final double heading = follower.getPose().getHeading(); // radians

        // ---- Controls (same mappings): LS Y=fwd/back, LS X=strafe, RS X=rotate ----
        double forward = shape(-gamepad1.left_stick_y);
        double strafe  = shape( gamepad1.left_stick_x);
        double rotate  = shape( gamepad1.right_stick_x);

        // ---- Heading-hold injection (only when driver not commanding rotation) ----
        if (Math.abs(gamepad1.right_stick_x) < ROT_DEADBAND) {
            double err = wrapAngle(headingTarget - heading);   // radians, -π..π
            double dt  = Math.max(1e-3, dtTimer.seconds());
            double deriv = (err - prevErr) / dt;
            double hold = kP_HOLD * err + kD_HOLD * deriv;
            rotate += hold;                 // inject correction
            prevErr = err;
        } else {
            headingTarget = heading;        // driver is steering; update target
            prevErr = 0.0;
        }
        dtTimer.reset();

        // ---- Drive (denominator normalization; same feel) ----
        mecanumDrive(forward, strafe, rotate);

        // ---- Manual launcher velocity control (unchanged) ----
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // ---- State machine: fire when RB pressed (unchanged) ----
        launch(gamepad1.right_bumper);

        // ---- Telemetry ----
        telemetry.addData("State", launchState);
        telemetry.addData("Drive", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);
        telemetry.addData("Heading(rad)", "%.3f", heading);
        telemetry.addData("Hold kP/kD", "%.3f / %.3f", kP_HOLD, kD_HOLD);
        telemetry.addData("Flywheel (vel)", "%.0f", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() { }

    // ---- Input shaping: deadband + expo (keeps same commands, smoother) ----
    private double shape(double v) {
        if (Math.abs(v) < DEADBAND) return 0.0;
        double s = (Math.abs(v) - DEADBAND) / (1.0 - DEADBAND);
        s = Math.copySign(s, v);
        return (1 - EXPO) * s + EXPO * s * s * s;
    }

    // ---- Wrap angle to [-π, π] ----
    private double wrapAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ---- Mecanum mixer (robot-centric, denominator normalization) ----
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denom = Math.max(1.0, abs(forward) + abs(strafe) + abs(rotate));

        double lf = (forward + strafe + rotate) / denom;
        double lb = (forward - strafe + rotate) / denom;
        double rf = (forward - strafe - rotate) / denom;
        double rb = (forward + strafe - rotate) / denom;

        // store for telemetry
        flPower = lf; frPower = rf; blPower = lb; brPower = rb;

        // apply to motors
        FrontLeftDrive.setPower(lf);
        FrontRightDrive.setPower(rf);
        BackLeftDrive.setPower(lb);
        BackRightDrive.setPower(rb);
    }

    // ---- Shooter state machine (unchanged) ----
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
