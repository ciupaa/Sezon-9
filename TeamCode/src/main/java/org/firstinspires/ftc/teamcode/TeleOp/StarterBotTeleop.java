/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 * (license text unchanged)
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
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

    ElapsedTime feederTimer = new ElapsedTime();

    // ---- Shooter state machine ----
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    // ---- Telemetry for mecanum ----
    double flPower, frPower, blPower, brPower;

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

        // ---- Motor directions (good baseline for mecanum) ----
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
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        // Feeders initial & direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {
        // ---- Drive: left stick Y = forward/back, left stick X = strafe, right stick X = rotate ----
        mecanumDrive(
                -gamepad1.left_stick_y,   // forward/back
                gamepad1.left_stick_x,   // strafe
                gamepad1.right_stick_x   // rotate
        );

        // ---- Manual launcher velocity control ----
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // ---- State machine: fire when RB pressed ----
        launch(gamepad1.right_bumper);

        // ---- Telemetry ----
        telemetry.addData("State", launchState);
        telemetry.addData("Drive", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);
        telemetry.addData("Flywheel (vel)", "%.0f", launcher.getVelocity());
    }

    @Override
    public void stop() { }

    // ---- Mecanum mixer (robot-centric) ----
    void mecanumDrive(double forward, double strafe, double rotate) {
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max; rf /= max; lb /= max; rb /= max;

        // store for telemetry
        flPower = lf; frPower = rf; blPower = lb; brPower = rb;

        // apply to motors
        FrontLeftDrive.setPower(lf);
        FrontRightDrive.setPower(rf);
        BackLeftDrive.setPower(lb);
        BackRightDrive.setPower(rb);
    }

    // ---- Shooter state machine ----
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
