package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Blue Close Gear Velocity")
public class GearVelocityBlueAuto extends OpMode {

    DcMotorEx flywheel;
    DcMotorEx flywheel2;

    private static final double MOTOR_TICKS_PER_REV        = 28.0;
    private static final double MOTOR_FREE_RPM              = 6000.0;

    // 1.0 = direct drive; 0.5 = flywheel 2x faster; 2.0 = flywheel half speed
    private static final double MOTOR_REV_PER_FLYWHEEL_REV = 1.0;

    private static final double SHOOT_SPEED_PERCENT         = 0.41;
    private static final double IDLE_SPEED_PERCENT          = 0.15;

    // Field-position speed curve
    private static final double FIELD_SPEED_A               = 0.0136112;
    private static final double FIELD_SPEED_B               = 0.528231;
    private static final double FIELD_SPEED_OFFSET          = 0.00;
    private static final double CLOSE_SHOT_SPEED_PERCENT    = 0.43;
    private static final double CLOSE_SHOT_TY_THRESHOLD     = 2.5;

    // Derived RPM constants
    private static final double MAX_FLYWHEEL_RPM        = MOTOR_FREE_RPM / MOTOR_REV_PER_FLYWHEEL_REV;
    private static final double SHOOT_FLYWHEEL_RPM      = MAX_FLYWHEEL_RPM * SHOOT_SPEED_PERCENT;
    private static final double IDLE_FLYWHEEL_RPM       = MAX_FLYWHEEL_RPM * IDLE_SPEED_PERCENT;
    private static final double CLOSE_SHOT_FLYWHEEL_RPM = MAX_FLYWHEEL_RPM * CLOSE_SHOT_SPEED_PERCENT;

    // Readiness / stability
    private static final double FLYWHEEL_READY_TOLERANCE_RPM  = 150.0;
    private static final double SHOOTER_STABLE_TIME_MS        = 175.0;
    private static final double TARGET_CHANGE_RESET_RPM       = 75.0;

    private static final double RAPID_FIRE_SHOT_SPACING_MS        = 575.0;
    private static final double TRIPLE_SHOT_FOLLOWUP_SPEED_SCALE  = 0.90;

    // PIDF gains
    private static final double SHOOTER_VELOCITY_P = 10.0;
    private static final double SHOOTER_VELOCITY_I = 3.0;
    private static final double SHOOTER_VELOCITY_D = 0.0;
    private static final double SHOOTER_VELOCITY_F = 11.7;

    private static final int SHOOTER_MODE_STOPPED       = 0;
    private static final int SHOOTER_MODE_IDLE          = 1;
    private static final int SHOOTER_MODE_FIELD_ADJUSTED = 2;

    // Shooter state
    private int    shooterMode                  = SHOOTER_MODE_STOPPED;
    private double targetFlywheelRpm            = 0.0;
    private double commandedSpeedPercent        = 0.0;
    private double lastAppliedTargetFlywheelRpm = 0.0;
    private boolean shooterWasReady             = false;
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();

    // Hardware
    Servo    hoodservo;
    Servo    slot1, slot2, slot3;
    DcMotor  intake;
    Servo    intake2;
    DcMotor  turretmotor;
    Limelight3A limelight;

    // Path following
    private Follower follower;
    private Paths    paths;
    private int      pathState = 0;

    // Turret PID  (integralPID is a field so it accumulates across loop() calls)
    double derivativeTx = 0;
    double lastTx       = 0;
    double lastTimeUpdated = 0;
    double pPID         = 0.015;
    double dPID         = 0.003;
    double iPID         = 0;
    double integralPID  = 0;   // FIX: was a local variable — now persists between loops

    // Intake pulse
    private boolean intakePulseActive  = false;
    private long    intakePulseStart   = 0;
    private boolean intakePulseEnabled = false;
    private long    intakePulseTime    = 600; // ms

    public void startIntakePulse() {
        if (!intakePulseEnabled) return;
        if (intakePulseActive) return;
        intakePulseActive = true;
        intakePulseStart  = System.currentTimeMillis();
        intake.setPower(1);
        intake2.setPosition(0);
    }

    public void updateIntakePulse() {
        if (!intakePulseActive) return;
        if (System.currentTimeMillis() - intakePulseStart >= intakePulseTime) {
            intake.setPower(-1);
            intake2.setPosition(1);
            intakePulseActive = false;
        }
    }

    /***************************
     *  Flicker state machine  *
     ***************************/
    private int     flickerState         = 0;
    private boolean flickerActive        = false;
    private long    flickerTimer         = 0;
    private long    flickerRecoveryStart = 0; // tracks when each ball was fed
    long upTime   = 500;
    long downTime = 400;

    public void startFlicker() {
        if (flickerActive) return;
        flickerActive = true;
        flickerState  = 0;
        flickerTimer  = System.currentTimeMillis();
    }

    public void updateFlicker() {
        if (!flickerActive) return;
        long now = System.currentTimeMillis();
        switch (flickerState) {

            // ── Ball 1 ───────────────────────────────────────────────────────
            case 0: // raise slot 1
                slot1.setPosition(0.47);
                flickerTimer = now;
                flickerState++;
                break;

            case 1: // hold up, then lower — ball 1 is now feeding into flywheel
                if (now - flickerTimer >= upTime) {
                    slot1.setPosition(0.98);
                    flickerRecoveryStart = now; // start timing flywheel recovery
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            // ── Recovery wait before ball 2 ──────────────────────────────────
            // Advance as soon as the flywheel is back at speed, OR after the
            // rapid-fire timeout — whichever comes first.
            case 2:
                if (shooterReady() ||
                        (now - flickerRecoveryStart >= RAPID_FIRE_SHOT_SPACING_MS)) {
                    flickerState++;
                }
                break;

            // ── Ball 2 ───────────────────────────────────────────────────────
            case 3: // raise slot 2
                slot2.setPosition(0.58);
                flickerTimer = now;
                flickerState++;
                break;

            case 4: // hold up, then lower
                if (now - flickerTimer >= upTime) {
                    slot2.setPosition(0.05);
                    flickerRecoveryStart = now;
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            // ── Recovery wait before ball 3 ──────────────────────────────────
            case 5:
                if (shooterReady() ||
                        (now - flickerRecoveryStart >= RAPID_FIRE_SHOT_SPACING_MS)) {
                    flickerState++;
                }
                break;

            // ── Ball 3 ───────────────────────────────────────────────────────
            case 6: // raise slot 3
                slot3.setPosition(0.63);
                flickerTimer = now;
                flickerState++;
                break;

            case 7: // hold up, then lower — sequence complete
                if (now - flickerTimer >= upTime) {
                    slot3.setPosition(0.12);
                    flickerActive = false;
                }
                break;
        }
    }

    public boolean flickerDone() {
        return !flickerActive;
    }

    /***************
     *    Paths    *
     ***************/
    // FIX: was referencing BlueAutoAlternateOrder.Paths — now correctly inner class of this file
    public static class Paths {
        public PathChain Path1, Path2, Path2b, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.5, 127.000), new Pose(58.000, 83.500)))
                    .setConstantHeadingInterpolation(Math.toRadians(144.5))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.000, 83.500), new Pose(15.000, 83.500)))
                    .setTangentHeadingInterpolation()
                    .build();

            // New path: from end of Path2 to (15, 70) with heading 90°
            Path2b = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 83.500), new Pose(15.000, 70.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 70.000), new Pose(58.000, 83.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58.000, 83.500), new Pose(48.000, 51.000), new Pose(11.5, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(11.5000, 60.000), new Pose(50, 54), new Pose(58.00, 83.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58, 83.500), new Pose(60.000, 27.000), new Pose(11.500, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.500, 35.000), new Pose(54.000, 109.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    // FIX: was referencing BlueAutoAlternateOrder.IntakeMode
    enum IntakeMode { INTAKE, OUTTAKE, OFF }

    private IntakeMode intakeMode = IntakeMode.INTAKE;

    /***************
     *    init()   *
     ***************/
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        paths    = new Paths(follower);   // FIX: no longer BlueAutoAlternateOrder.Paths
        follower.setStartingPose(new Pose(20.5, 127, Math.toRadians(144.5)));

        intake       = hardwareMap.get(DcMotor.class,  "intakemotor");
        intake2      = hardwareMap.get(Servo.class,    "intake2servo");
        turretmotor  = hardwareMap.get(DcMotor.class,  "turretmotor");

        flywheel  = hardwareMap.get(DcMotorEx.class, "testemotor");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheelmotor2");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setVelocityPIDFCoefficients(
                SHOOTER_VELOCITY_P, SHOOTER_VELOCITY_I,
                SHOOTER_VELOCITY_D, SHOOTER_VELOCITY_F);
        flywheel2.setVelocityPIDFCoefficients(
                SHOOTER_VELOCITY_P, SHOOTER_VELOCITY_I,
                SHOOTER_VELOCITY_D, SHOOTER_VELOCITY_F);

        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(0.68);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setMaxPower(0.95);
        lastTimeUpdated = System.nanoTime();

        slot1 = hardwareMap.get(Servo.class, "lift1");
        slot2 = hardwareMap.get(Servo.class, "lift2");
        slot3 = hardwareMap.get(Servo.class, "lift3");
        slot1.setPosition(0.98);
        slot2.setPosition(0.05);
        slot3.setPosition(0.12);

        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
    }

    /***************
     *    loop()   *
     ***************/
    public void loop() {
        follower.update();
        updateFlicker();

        // Intake control (skipped if pulse is active)
        if (!intakePulseActive) {
            switch (intakeMode) {
                case INTAKE:
                    intake.setPower(-1);
                    intake2.setPosition(1);
                    break;
                case OUTTAKE:
                    intake.setPower(1);
                    intake2.setPosition(0);
                    break;
                case OFF:
                    intake.setPower(0);
                    intake2.setPosition(0.5);
                    break;
            }
        }

        // Turret PID
        // FIX: removed local "double iPID = 0" and "double integralPID = 0" that shadowed
        //      the class fields, causing the integral to reset to 0 every loop iteration.
        LLResult result    = limelight.getLatestResult();
        boolean  targetFound  = false;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                telemetry.addData("FiducialID", fr.getFiducialId());
                if (fr.getFiducialId() == 20) {
                    double tx = fr.getTargetXDegrees();
                    double dt = System.nanoTime() - lastTimeUpdated;
                    derivativeTx = 1_000_000_000.0 * (tx - lastTx) / dt;
                    integralPID += tx * dt / 1_000_000_000.0;  // now actually accumulates
                    turretmotor.setPower(pPID * tx + dPID * derivativeTx + iPID * integralPID);
                    targetFound     = true;
                    lastTx          = tx;
                    lastTimeUpdated = System.nanoTime();
                }
            }
        }
        if (!targetFound) {
            turretmotor.setPower(0);
            integralPID = 0;  // reset integral when target is lost so it doesn't wind up
            telemetry.addData("Limelight", "No data available");
        } else {
            telemetry.addData("Limelight", "Data available");
        }

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Telemetry
        double fw1Rpm = motorTicksPerSecondToFlywheelRpm(Math.abs(flywheel.getVelocity()));
        double fw2Rpm = motorTicksPerSecondToFlywheelRpm(Math.abs(flywheel2.getVelocity()));
        telemetry.addData("----- Shooter -----",       null);
        telemetry.addData("Mode",                      shooterModeName());
        telemetry.addData("Commanded Speed %",         commandedSpeedPercent);
        telemetry.addData("Target RPM",                targetFlywheelRpm);
        telemetry.addData("Flywheel 1 RPM",            fw1Rpm);
        telemetry.addData("Flywheel 2 RPM",            fw2Rpm);
        telemetry.addData("Shooter Ready",             shooterReady());
        telemetry.addData("Target ticks/s",            flywheelRpmToMotorTicksPerSecond(targetFlywheelRpm));
        telemetry.addData("Motor 1 actual ticks/s",    flywheel.getVelocity());
        telemetry.addData("Motor 2 actual ticks/s",    flywheel2.getVelocity());
        telemetry.addData("Flywheel 1 encoder pos",    flywheel.getCurrentPosition());
        telemetry.addData("Flywheel 2 encoder pos",    flywheel2.getCurrentPosition());
        telemetry.addData("Rapid Shot Spacing ms",     RAPID_FIRE_SHOT_SPACING_MS);
        telemetry.addData("Followup Speed Scale",      TRIPLE_SHOT_FOLLOWUP_SPEED_SCALE);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    /***********************
     *  Shooter helpers    *
     ***********************/

    private double calculateFieldAdjustedSpeedPercent(double ty) {
        double sp = (ty >= CLOSE_SHOT_TY_THRESHOLD)
                ? CLOSE_SHOT_SPEED_PERCENT
                : FIELD_SPEED_A * ty + FIELD_SPEED_B + FIELD_SPEED_OFFSET;
        return Math.max(0.0, Math.min(1.0, sp));
    }

    private String shooterModeName() {
        if (shooterMode == SHOOTER_MODE_IDLE)           return "IDLE";
        if (shooterMode == SHOOTER_MODE_FIELD_ADJUSTED) return "FIELD_ADJUSTED";
        return "STOPPED";
    }

    private double flywheelRpmToMotorRpm(double flywheelRpm) {
        return flywheelRpm * MOTOR_REV_PER_FLYWHEEL_REV;
    }

    private double motorRpmToFlywheelRpm(double motorRpm) {
        return motorRpm / MOTOR_REV_PER_FLYWHEEL_REV;
    }

    private double flywheelRpmToMotorTicksPerSecond(double flywheelRpm) {
        return flywheelRpmToMotorRpm(flywheelRpm) * MOTOR_TICKS_PER_REV / 60.0;
    }

    private double motorTicksPerSecondToFlywheelRpm(double motorTicksPerSecond) {
        return motorRpmToFlywheelRpm(motorTicksPerSecond * 60.0 / MOTOR_TICKS_PER_REV);
    }

    // FIX: commandedSpeedPercent is now updated here so telemetry is always accurate.
    // FIX: motors now use setVelocity() instead of setPower() so the PIDF loop is active.
    private void setShooterFlywheelRpm(double flywheelRpm) {
        targetFlywheelRpm    = clipFlywheelRpmToMotorLimit(flywheelRpm);
        commandedSpeedPercent = targetFlywheelRpm / MAX_FLYWHEEL_RPM;   // FIX: was never set

        if (Math.abs(targetFlywheelRpm - lastAppliedTargetFlywheelRpm) > TARGET_CHANGE_RESET_RPM) {
            shooterWasReady = false;
            shooterReadyTimer.reset();
        }
        lastAppliedTargetFlywheelRpm = targetFlywheelRpm;

        double motorTicksPerSecond = flywheelRpmToMotorTicksPerSecond(targetFlywheelRpm);
        flywheel.setVelocity(motorTicksPerSecond);   // FIX: was setPower()
        flywheel2.setVelocity(motorTicksPerSecond);  // FIX: was setPower()
    }

    private double clipFlywheelRpmToMotorLimit(double flywheelRpm) {
        if (MOTOR_REV_PER_FLYWHEEL_REV <= 0) return 0.0;
        double maxRpm = MOTOR_FREE_RPM / MOTOR_REV_PER_FLYWHEEL_REV;
        return Math.max(0.0, Math.min(flywheelRpm, maxRpm));
    }

    private boolean shooterReady() {
        if (targetFlywheelRpm < 100.0) {
            shooterWasReady = false;
            shooterReadyTimer.reset();
            return false;
        }
        double fw1Rpm  = motorTicksPerSecondToFlywheelRpm(Math.abs(flywheel.getVelocity()));
        double fw2Rpm  = motorTicksPerSecondToFlywheelRpm(Math.abs(flywheel2.getVelocity()));
        boolean ready1 = Math.abs(fw1Rpm - targetFlywheelRpm) < FLYWHEEL_READY_TOLERANCE_RPM;
        boolean ready2 = Math.abs(fw2Rpm - targetFlywheelRpm) < FLYWHEEL_READY_TOLERANCE_RPM;

        if (!ready1 || !ready2) {
            shooterWasReady = false;
            shooterReadyTimer.reset();
            return false;
        }
        if (!shooterWasReady) {
            shooterWasReady = true;
            shooterReadyTimer.reset();
        }
        return shooterReadyTimer.milliseconds() >= SHOOTER_STABLE_TIME_MS;
    }

    /*******************************
     *  Autonomous path update     *
     *******************************/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            // ── Start spinning up and drive to hub ─────────────────────────────
            case 0:
                shooterMode = SHOOTER_MODE_FIELD_ADJUSTED;
                setShooterFlywheelRpm(CLOSE_SHOT_FLYWHEEL_RPM);
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            // ── Wait for path done AND flywheel at speed, then fire ────────────
            case 1:
                // FIX: was a flat 2000 ms wait; now waits for actual RPM readiness
                if (!follower.isBusy() && shooterReady()) {
                    startFlicker();
                    pathState = 101;
                }
                break;

            case 101:
                if (flickerDone()) {
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;

            // ── At pickup → run new Path2b to (15, 70) heading 90° ──────────
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2b, true);
                    pathState = 3;
                }
                break;

            // ── At (15, 70) → return to hub ───────────────────────────────────
            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path3, true);
                    pathState = 4;
                }
                break;

            // ── At hub: wait for ready, fire ──────────────────────────────────
            case 4:
                if (!follower.isBusy() && shooterReady()) {
                    startFlicker();
                    pathState = 401;
                }
                break;

            case 401:
                if (flickerDone()) {
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path4, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path5, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy() && shooterReady()) {
                    startFlicker();
                    pathState = 601;
                }
                break;

            case 601:
                if (flickerDone()) {
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path6, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path7, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy() && shooterReady()) {
                    startFlicker();
                    pathState = 801;
                }
                break;

            case 801:
                if (flickerDone()) {
                    shooterMode = SHOOTER_MODE_STOPPED;
                    setShooterFlywheelRpm(0);
                    pathState = 9;
                }
                break;

            case 9:
                // Auto complete — do nothing
                break;
        }
    }
}