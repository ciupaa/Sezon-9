package org.firstinspires.ftc.teamcode.Autonomie;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAuto", group="StarterBot")
//@Disabled
public class StarterBotAuto extends OpMode {

    // -------- Shooter / Feeder -------
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2.0;

    // -------- Drive geometry & kinematics -------
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404; // distance between left/right wheel centers

    // how close (in mm) we must be to target before "holdSeconds" timer can elapse
    final double POSITION_TOLERANCE_MM = 10;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    // -------- Timers -------
    private final ElapsedTime shotTimer   = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer  = new ElapsedTime();

    // -------- Hardware -------
    private DcMotor lf = null; // fata_stanga
    private DcMotor rf = null; // fata_dreapta
    private DcMotor lb = null; // spate_stanga
    private DcMotor rb = null; // spate-dreapta

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // -------- State machines -------
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState;

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE
    }
    private AutonomousState autonomousState;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        // Map hardware (names must match RC config)
        lf = hardwareMap.get(DcMotor.class, "fata_stanga");
        rf = hardwareMap.get(DcMotor.class, "fata_dreapta");
        lb = hardwareMap.get(DcMotor.class, "spate_stanga");
        rb = hardwareMap.get(DcMotor.class, "spate-dreapta"); // note the hyphen

        launcher   = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder= hardwareMap.get(CRServo.class, "right_feeder");

        // Directions for mecanum (same as TeleOp baseline)
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // Brake for crisp stops
        lf.setZeroPowerBehavior(BRAKE);
        rf.setZeroPowerBehavior(BRAKE);
        lb.setZeroPowerBehavior(BRAKE);
        rb.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Reset encoders before we start
        resetAllDriveEncoders();

        // Launcher config
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        // Feeders initial & direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Let driver choose alliance before start
        if (gamepad1.b) alliance = Alliance.RED;
        else if (gamepad1.x) alliance = Alliance.BLUE;

        // keep servos idle
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
        // nothing special
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        resetAllDriveEncoders();
                        launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                // drive backward 4 inches to clear goal
                if (drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1.0)) {
                    resetAllDriveEncoders();
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                robotRotationAngle = (alliance == Alliance.RED) ? 45 : -45;
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1.0)) {
                    resetAllDriveEncoders();
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                // back up 26 inches to get off the start line
                if (drive(DRIVE_SPEED, -26, DistanceUnit.INCH, 1.0)) {
                    autonomousState = AutonomousState.COMPLETE;
                    stopAllDrive();
                }
                break;

            case COMPLETE:
                // do nothing
                stopAllDrive();
                break;
        }

        // Telemetry
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("FL/FR Pos", "%d / %d", lf.getCurrentPosition(), rf.getCurrentPosition());
        telemetry.addData("BL/BR Pos", "%d / %d", lb.getCurrentPosition(), rb.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() { }

    // -------- Shooter state machine ----------
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;

            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;

            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    // -------- Driving helpers (mecanum, encoder RUN_TO_POSITION) ----------

    private void resetAllDriveEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTimer.reset();
    }

    private void stopAllDrive() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    /**
     * Drive straight (forward +, backward -) a specified distance using all four motors.
     * @param speed 0..1
     * @param distance distance in given unit (positive = forward)
     * @param distanceUnit INCH / MM etc.
     * @param holdSeconds time to hold inside tolerance before returning true
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double targetMm = distanceUnit.toMm(distance);
        final int targetTicks = (int) Math.round(targetMm * TICKS_PER_MM);

        lf.setTargetPosition(targetTicks);
        rf.setTargetPosition(targetTicks);
        lb.setTargetPosition(targetTicks);
        rb.setTargetPosition(targetTicks);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        // if any wheel is outside tolerance, reset the hold timer
        if (!withinPositionToleranceMm(targetMm)) {
            driveTimer.reset();
        }
        return driveTimer.seconds() > holdSeconds;
    }

    /**
     * In-place rotation by angle (+CCW, -CW). Left wheels go opposite of right wheels.
     * @param speed 0..1
     * @param angle requested rotation
     * @param angleUnit DEGREES/RADIANS
     * @param holdSeconds time to hold inside tolerance before returning true
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double radians = angleUnit.toRadians(angle);
        final double arcMm = radians * (TRACK_WIDTH_MM / 2.0); // arc length per side
        final int ticks = (int) Math.round(arcMm * TICKS_PER_MM);

        // left side negative, right side positive for CCW (+angle)
        lf.setTargetPosition(-ticks);
        lb.setTargetPosition(-ticks);
        rf.setTargetPosition(+ticks);
        rb.setTargetPosition(+ticks);

        lf.setPower(speed);
        rf.setPower(speed);
        lb.setPower(speed);
        rb.setPower(speed);

        // use side errors to gate the hold timer
        if (!withinRotationToleranceMm(arcMm)) {
            driveTimer.reset();
        }
        return driveTimer.seconds() > holdSeconds;
    }

    // -------- Tolerance helpers --------

    private boolean withinPositionToleranceMm(double targetMm) {
        final double tolTicks = POSITION_TOLERANCE_MM * TICKS_PER_MM;
        int tgt = (int) Math.round(targetMm * TICKS_PER_MM);

        int eLF = Math.abs(tgt - lf.getCurrentPosition());
        int eRF = Math.abs(tgt - rf.getCurrentPosition());
        int eLB = Math.abs(tgt - lb.getCurrentPosition());
        int eRB = Math.abs(tgt - rb.getCurrentPosition());

        return eLF < tolTicks && eRF < tolTicks && eLB < tolTicks && eRB < tolTicks;
    }

    private boolean withinRotationToleranceMm(double arcMmPerSide) {
        final int tgtLeft  = (int) Math.round(-arcMmPerSide * TICKS_PER_MM);
        final int tgtRight = (int) Math.round(+arcMmPerSide * TICKS_PER_MM);
        final int tolTicks = (int) Math.round(POSITION_TOLERANCE_MM * TICKS_PER_MM);

        int eLF = Math.abs(tgtLeft  - lf.getCurrentPosition());
        int eLB = Math.abs(tgtLeft  - lb.getCurrentPosition());
        int eRF = Math.abs(tgtRight - rf.getCurrentPosition());
        int eRB = Math.abs(tgtRight - rb.getCurrentPosition());

        return eLF < tolTicks && eLB < tolTicks && eRF < tolTicks && eRB < tolTicks;
    }
}
