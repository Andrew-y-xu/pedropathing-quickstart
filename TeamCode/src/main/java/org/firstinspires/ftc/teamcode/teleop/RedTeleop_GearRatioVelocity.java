package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.IndicatorLight;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Red Teleop Western Edge Use This One")
public class RedTeleop_GearRatioVelocity extends OpMode {

    DcMotorEx testmotor;
    DcMotorEx flywheelmotor2;
    DcMotor intakemotor;

    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;
    Servo liftservo;
    Servo lift2servo;
    Servo lift3servo;
    Servo intake2servo;
    Servo hoodservo;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

    int cycleMode = 0; // 0 = none, 1 = lift1, 2 = lift2, 3 = lift3

    private boolean cycleRunning = false;
    private double cycleStartTime = 0;
    private boolean tripleShotRunning = false;
    private int tripleShotIndex = 0;
    private boolean tripleShotButtonWasPressed = false;
    private double tripleShotLastFeedTime = 0;
    private final int[] tripleShotModes = {2, 1, 3}; //shot order 1 is left, 2 is right, 3 is back
    DcMotor poopeemotorey;
    private Limelight3A lookylookyseesee;
    ElapsedTime timer = new ElapsedTime();

    private boolean rtWasPressed = false;
    private boolean ltWasPressed = false;

    double pPID = 0.011;
    double dPID = 0.003;
    double iPID = 0;
    double lastTx = 0;
    double derivativeTx = 0;
    double lastTimeUpdated = 0;

    //--- AutoShoot
    double limelight_tx = 0;
    double limelightTy = 0;
    private boolean aButtonWasPressed = false;
    private boolean bButtonWasPressed = false;

    double servoPositionValue = 0;

    // Motor-specific values. Change these if the shooter motor changes.
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_FREE_RPM = 6000.0;

    // Gear ratio definition:
    // motor revolutions per 1 flywheel revolution.
    // 1.0 = direct drive
    // 0.5 = flywheel spins 2x faster than motor
    // 2.0 = flywheel spins half motor speed
    private static final double MOTOR_REV_PER_FLYWHEEL_REV = 1.0;

    // Desired shooter speeds as a percentage of max flywheel RPM.
    // With a 6000 RPM motor and 1:1 gearing:
    // 0.46 = 2760 flywheel RPM on a 6000 RPM motor with 1:1 gearing.
    // Lower target lets the flickers release sooner and reduces shot speed.
    private static final double SHOOT_SPEED_PERCENT = 0.41;
    private static final double IDLE_SPEED_PERCENT = 0.15;

    // Old field-position speed curve, converted from power percentage to RPM percentage.
    // The base is lowered from the old power curve so velocity control does not overshoot shot speed.
    private static final double FIELD_SPEED_A = 0.0136112;
    private static final double FIELD_SPEED_B = 0.528231;
    private static final double FIELD_SPEED_OFFSET = 0.00;
    private static final double CLOSE_SHOT_SPEED_PERCENT = 0.43;
    private static final double CLOSE_SHOT_TY_THRESHOLD = 2.5;

    // Computed from motor speed and gear ratio so motor/gearing changes only need edits above.
    private static final double MAX_FLYWHEEL_RPM = MOTOR_FREE_RPM / MOTOR_REV_PER_FLYWHEEL_REV;
    private static final double SHOOT_FLYWHEEL_RPM = MAX_FLYWHEEL_RPM * SHOOT_SPEED_PERCENT;
    private static final double IDLE_FLYWHEEL_RPM = MAX_FLYWHEEL_RPM * IDLE_SPEED_PERCENT;

    // Ball feed is blocked until both flywheels are close to target and stable.
    // Tightened from the rapid-fire test values because the first shot was releasing too early.
    private static final double FLYWHEEL_READY_TOLERANCE_RPM = 150.0;
    private static final double SHOOTER_STABLE_TIME_MS = 175.0;
    private static final double TARGET_CHANGE_RESET_RPM = 75.0;

    // Triple-shot should be rapid and guaranteed to finish. The first shot waits for normal
    // shooterReady(); shots 2 and 3 fire when ready OR after this spacing from the prior shot.
    private static final double RAPID_FIRE_SHOT_SPACING_MS = 575.0;

    // After ball 1, lower the commanded RPM so balls 2 and 3 do not come out hotter.
    // 0.90 means shots 2/3 use 90% of the normal field-adjusted RPM target.
    private static final double TRIPLE_SHOT_FOLLOWUP_SPEED_SCALE = 0.90;

    // Starting PIDF values. Tune these on the robot.
    private static final double SHOOTER_VELOCITY_P = 10.0;
    private static final double SHOOTER_VELOCITY_I = 3.0;
    private static final double SHOOTER_VELOCITY_D = 0.0;
    private static final double SHOOTER_VELOCITY_F = 11.7;

    private static final int SHOOTER_MODE_STOPPED = 0;
    private static final int SHOOTER_MODE_IDLE = 1;
    private static final int SHOOTER_MODE_FIELD_ADJUSTED = 2;

    private int shooterMode = SHOOTER_MODE_STOPPED;
    private double targetFlywheelRpm = 0.0;
    private double commandedSpeedPercent = 0.0;
    private double lastAppliedTargetFlywheelRpm = 0.0;
    private boolean shooterWasReady = false;
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();

    double servo_value = 0.5;
    String limelightMessage = "No data available";
    String autoShootMessage = "Shooter At Init";

    double deadzone = 0.05;
    private boolean jerkRunning = false;
    private double jerkStartTime = 0;
    private ElapsedTime jerkTimer = new ElapsedTime();

    private double value = 0;
    public double speed = 1.0;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static final String SENSOR3_NAME = "color3";
    private static final String SENSOR2_NAME = "color2";
    private static final String SENSOR1_NAME = "color1";

    private static final String INDICATOR3_NAME = "indicator3";
    private static final String INDICATOR2_NAME = "indicator2";
    private static final String INDICATOR1_NAME = "indicator1";

    private NormalizedColorSensor colorSensor3;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor1;
    private double colorPauseEnd1 = 0;
    private double colorPauseEnd2 = 0;
    private double colorPauseEnd3 = 0;
    private boolean pauseColor1 = false;
    private boolean pauseColor2 = false;
    private boolean pauseColor3 = false;
    private IndicatorLight light3;
    private IndicatorLight light2;
    private IndicatorLight light1;

    private String lastLockedColor3 = "unknown";
    private String lastLockedColor2 = "unknown";
    private String lastLockedColor1 = "unknown";

    @Override
    public void init() {
        testmotor      = hardwareMap.get(DcMotorEx.class, "testemotor");
        flywheelmotor2 = hardwareMap.get(DcMotorEx.class, "flywheelmotor2");

        testmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        testmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        testmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelmotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_frontLeft  = hardwareMap.get(DcMotorEx.class, "lf");
        motor_frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        motor_backLeft   = hardwareMap.get(DcMotorEx.class, "lr");
        motor_backRight  = hardwareMap.get(DcMotorEx.class, "rr");

        intakemotor = hardwareMap.dcMotor.get("intakemotor");
        hoodservo   = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        poopeemotorey    = hardwareMap.get(DcMotor.class, "turretmotor");
        lookylookyseesee = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        lookylookyseesee.pipelineSwitch(0);
        lookylookyseesee.start();

        motor_frontLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_backRight.setDirection(DcMotor.Direction.REVERSE);
        motor_frontRight.setDirection(DcMotor.Direction.FORWARD);
        motor_backLeft.setDirection(DcMotor.Direction.REVERSE);

        motor_backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor_frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor_frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftservo    = hardwareMap.get(Servo.class, "lift2");
        lift2servo   = hardwareMap.get(Servo.class, "lift1");
        lift3servo   = hardwareMap.get(Servo.class, "lift3");
        intake2servo = hardwareMap.get(Servo.class, "intake2servo");

        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelmotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);

        testmotor.setVelocityPIDFCoefficients(
                SHOOTER_VELOCITY_P,
                SHOOTER_VELOCITY_I,
                SHOOTER_VELOCITY_D,
                SHOOTER_VELOCITY_F
        );
        flywheelmotor2.setVelocityPIDFCoefficients(
                SHOOTER_VELOCITY_P,
                SHOOTER_VELOCITY_I,
                SHOOTER_VELOCITY_D,
                SHOOTER_VELOCITY_F
        );

        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, SENSOR3_NAME);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, SENSOR2_NAME);
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, SENSOR1_NAME);

        light3 = new IndicatorLight(hardwareMap, INDICATOR3_NAME);
        light2 = new IndicatorLight(hardwareMap, INDICATOR2_NAME);
        light1 = new IndicatorLight(hardwareMap, INDICATOR1_NAME);

        liftservo.setPosition(0.05);
        lift2servo.setPosition(0.98);
        lift3servo.setPosition(0.12);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void frontLeft(double power)  { if (power > 1) power = 1; motor_frontLeft.setPower(power); }
    public void frontRight(double power) { if (power > 1) power = 1; motor_frontRight.setPower(power); }
    public void backLeft(double power)   { if (power > 1) power = 1; motor_backLeft.setPower(power); }
    public void backRight(double power)  { if (power > 1) power = 1; motor_backRight.setPower(power); }

    @Override
    public void loop() {
        booleanIncrementer = 0;
        boolean gamePad1yIsPressed = ifPressed(gamepad1.y);
        boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        boolean gamePad1bIsPressed = ifPressed(gamepad1.b);

        /***************************/
        /****Drive Train Related ***/
        /***************************/
        double vx = speed * (-gamepad1.left_stick_y * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double vy = speed * ( gamepad1.left_stick_x  * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double o  = speed * ( gamepad1.right_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));

        if (gamePad1yIsPressed) speed = 0.7;
        if (gamePad1bIsPressed) speed = 0.3;
        if (gamePad1aIsPressed) speed = 1.5;

        double leftFrontPower  = vx + vy + o;
        double rightFrontPower = vx - vy - o;
        double leftBackPower   = vx - vy + o;
        double rightBackPower  = vx + vy - o;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        frontLeft(leftFrontPower);
        frontRight(rightFrontPower);
        backLeft(leftBackPower);
        backRight(rightBackPower);

        telemetry.addData("frontLeft",  leftFrontPower);
        telemetry.addData("frontRight", rightFrontPower);
        telemetry.addData("backLeft",   leftBackPower);
        telemetry.addData("backRight",  rightBackPower);

        /***************************/
        /****Jerk ***/
        /***************************/
        if (gamepad1.x && !jerkRunning) {
            jerkRunning = true;
            jerkStartTime = jerkTimer.milliseconds();
        }

        if (jerkRunning) {
            double t = jerkTimer.milliseconds() - jerkStartTime;

            if (t < 210) {
                frontLeft(1.0); frontRight(1.0); backLeft(1.0); backRight(1.0);
            } else if (t < 400) {
                frontLeft(-1.0); frontRight(-1.0); backLeft(-1.0); backRight(-1.0);
            } else if (t < 420) {
                frontLeft(0.9); backLeft(0.9); frontRight(-0.9); backRight(-0.9);
            } else {
                jerkRunning = false;
                frontLeft(0); frontRight(0); backLeft(0); backRight(0);
            }
        }

        /***************************/
        /**** Lift Cycles ***/
        /***************************/
        if (gamepad2.dpad_left && !cycleRunning) {
            if (shooterReady()) {
                cycleRunning = true;
                cycleMode = 1;
                cycleStartTime = timer.milliseconds();
            } else {
                autoShootMessage = "WAIT - shooter not at speed";
            }
        }
        if (gamepad2.dpad_right && !cycleRunning) {
            if (shooterReady()) {
                cycleRunning = true;
                cycleMode = 2;
                cycleStartTime = timer.milliseconds();
            } else {
                autoShootMessage = "WAIT - shooter not at speed";
            }
        }
        if (gamepad2.dpad_down && !cycleRunning) {
            if (shooterReady()) {
                cycleRunning = true;
                cycleMode = 3;
                cycleStartTime = timer.milliseconds();
            } else {
                autoShootMessage = "WAIT - shooter not at speed";
            }
        }

        boolean tripleShotButtonPressed = gamepad2.left_trigger > 0.5;
        if (tripleShotButtonPressed && !tripleShotButtonWasPressed && !tripleShotRunning) {
            tripleShotRunning = true;
            tripleShotIndex = 0;
            tripleShotLastFeedTime = 0;
            shooterMode = SHOOTER_MODE_FIELD_ADJUSTED;
            autoShootMessage = "Triple shot started";
        }
        tripleShotButtonWasPressed = tripleShotButtonPressed;

        if (gamepad2.back) {
            tripleShotRunning = false;
            tripleShotIndex = 0;
            autoShootMessage = "Triple shot cancelled";
        }

        if (tripleShotRunning && !cycleRunning) {
            if (tripleShotIndex >= tripleShotModes.length) {
                tripleShotRunning = false;
                autoShootMessage = "Triple shot complete";
            } else {
                boolean firstShot = tripleShotIndex == 0;
                boolean rapidSpacingMet = !firstShot &&
                        timer.milliseconds() - tripleShotLastFeedTime >= RAPID_FIRE_SHOT_SPACING_MS;

                if (shooterReady() || rapidSpacingMet) {
                    cycleRunning = true;
                    cycleMode = tripleShotModes[tripleShotIndex];
                    cycleStartTime = timer.milliseconds();
                    tripleShotLastFeedTime = cycleStartTime;
                    tripleShotIndex++;
                    autoShootMessage = "Triple shot feeding ball " + tripleShotIndex;
                } else {
                    autoShootMessage = firstShot
                            ? "Triple shot waiting for first-shot RPM"
                            : "Triple shot rapid waiting";
                }
            }
        }

        if (cycleRunning) {
            double t = timer.milliseconds() - cycleStartTime;

            if (cycleMode == 1) {
                if      (t < 300) liftservo.setPosition(0.58);
                else if (t < 500) {
                    liftservo.setPosition(0.05);
                    lastLockedColor1 = "unknown";
                    light1.white();
                    pauseColor1 = true;
                    colorPauseEnd1 = timer.milliseconds() + 2500;
                }
                else { cycleRunning = false; cycleMode = 0; }
            } else if (cycleMode == 2) {
                if (t < 300) lift2servo.setPosition(0.47);
                else if (t < 500) {
                    lift2servo.setPosition(0.98);
                    lastLockedColor2 = "unknown";
                    light2.white();
                    pauseColor2 = true;
                    colorPauseEnd2 = timer.milliseconds() + 2500;
                }
                else { cycleRunning = false; cycleMode = 0; }
            } else if (cycleMode == 3) {
                if (t < 300) lift3servo.setPosition(0.63);
                else if (t < 500) {
                    lift3servo.setPosition(0.12);
                    lastLockedColor3 = "unknown";
                    light3.white();
                    pauseColor3 = true;
                    colorPauseEnd3 = timer.milliseconds() + 2500;
                }
                else { cycleRunning = false; cycleMode = 0; }
            }
        }

        /***************************/
        /**** Intake ***/
        /***************************/
        if (gamepad2.b || gamepad1.x) {
            intakemotor.setPower(0);
            intake2servo.setPosition(0.5);
        }
        if (gamepad2.y || gamepad1.dpad_up) {
            intakemotor.setPower(1.0);
            intake2servo.setPosition(1.0);
        }
        if (gamepad2.x || gamepad1.right_bumper || gamepad1.left_bumper) {
            intakemotor.setPower(-1.0);
            intake2servo.setPosition(0.0);
        }

        value = Math.max(0.0, Math.min(value, 1.0));

        /***************************/
        /**** Flywheel Velocity Control ***/
        /***************************/

        if (gamepad2.left_bumper) {
            shooterMode = SHOOTER_MODE_FIELD_ADJUSTED;
            autoShootMessage = "Shooter field-adjusted RPM";
        } else if (gamepad2.a && !aButtonWasPressed) {
            shooterMode = SHOOTER_MODE_IDLE;
            autoShootMessage = "Shooter idle RPM";
        } else if (gamepad2.right_bumper) {
            shooterMode = SHOOTER_MODE_STOPPED;
            autoShootMessage = "Shooter stopped";
        }

        aButtonWasPressed = gamepad2.a;
        bButtonWasPressed = gamepad2.right_bumper;

        /***************************/
        /**** Limelight / Auto Aim ***/
        /***************************/
        LLResult resultsofpooe = lookylookyseesee.getLatestResult();
        boolean doesiseeitfoundboi = false;
        double limelight_tx_local = 0;
        double limelightTy_local  = 0;
        double dt = 0;
        double integralPID = 0;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
                telemetry.addData("FiducialID", fr.getFiducialId());
                limelight_tx_local = fr.getTargetXDegrees();
                limelightTy_local  = fr.getTargetYDegrees();

                if (fr.getFiducialId() == 24) {
                    dt = System.nanoTime() - lastTimeUpdated;
                    derivativeTx = 1000000000.0 * (limelight_tx_local - lastTx) / (dt);
                    integralPID += limelight_tx_local * dt / 1000000000;
                    poopeemotorey.setPower(pPID * limelight_tx_local + dPID * derivativeTx + iPID * integralPID);
                    doesiseeitfoundboi = true;
                    lastTx = limelight_tx_local;
                    lastTimeUpdated = System.nanoTime();
                }
            }
        }

        if (!doesiseeitfoundboi) {
            telemetry.addData("Limelight", "No data available");
        } else {
            telemetry.addData("Limelight", "Data available");
        }

        if (shooterMode == SHOOTER_MODE_STOPPED) {
            commandedSpeedPercent = 0.0;
            setShooterFlywheelRpm(0.0);
        } else if (shooterMode == SHOOTER_MODE_IDLE) {
            commandedSpeedPercent = IDLE_SPEED_PERCENT;
            setShooterFlywheelRpm(IDLE_FLYWHEEL_RPM);
        } else {
            if (doesiseeitfoundboi) {
                commandedSpeedPercent = calculateFieldAdjustedSpeedPercent(limelightTy_local);
                autoShootMessage = "Shooter field-adjusted RPM";
            } else {
                commandedSpeedPercent = SHOOT_SPEED_PERCENT;
                autoShootMessage = "Shooter fallback RPM - no target";
            }

            commandedSpeedPercent = applyTripleShotFollowupSpeedScale(commandedSpeedPercent);
            setShooterFlywheelRpm(MAX_FLYWHEEL_RPM * commandedSpeedPercent);
        }

        /***************************/
        /**** Apply Turret ***/
        /***************************/
        if (!doesiseeitfoundboi) {
            // Manual turret control when Limelight does not have the target.
            double turretPower = 0.0;
            if (gamepad1.left_trigger > 0.05) {
                turretPower =  gamepad1.left_trigger * 0.35;
            } else if (gamepad1.right_trigger > 0.05) {
                turretPower = -gamepad1.right_trigger * 0.35;
            }
            poopeemotorey.setPower(turretPower);
        }

        /***************************/
        /**** Color Sensors ***/
        /***************************/
        String detected3 = "unknown";
        String detected2 = "unknown";
        String detected1 = "unknown";

        if (!pauseColor3 || timer.milliseconds() > colorPauseEnd3) {
            pauseColor3 = false;
            detected3 = detectColor(colorSensor3);
        }
        if (!pauseColor2 || timer.milliseconds() > colorPauseEnd2) {
            pauseColor2 = false;
            detected2 = detectColor(colorSensor2);
        }
        if(!pauseColor1 || timer.milliseconds() > colorPauseEnd1){
            pauseColor1 = false;
            detected1 = detectColor(colorSensor1);
        }

        if (!pauseColor3 && (detected3.equals("green") || detected3.equals("purple"))) {
            lastLockedColor3 = detected3;
        }
        if (!pauseColor2 && (detected2.equals("green") || detected2.equals("purple"))) {
            lastLockedColor2 = detected2;
        }
        if(!pauseColor1 && (detected1.equals("green") || detected1.equals("purple"))){
            lastLockedColor1= detected1;
        }

        if      (pauseColor3)                         light3.white();
        else if (lastLockedColor3.equals("green"))    light3.green();
        else if (lastLockedColor3.equals("purple"))   light3.violet();

        if      (pauseColor2)                         light2.white();
        else if (lastLockedColor2.equals("green"))    light2.green();
        else if (lastLockedColor2.equals("purple"))   light2.violet();

        if      (pauseColor1)                         light1.white();
        else if (lastLockedColor1.equals("green"))    light1.green();
        else if (lastLockedColor1.equals("purple"))  light1.violet();

        /***************************/
        /**** Telemetry ***/
        /***************************/
        telemetry.addData("Sensor3 Detected",  detected3);
        telemetry.addData("Sensor3 Locked",    lastLockedColor3);
        telemetry.addData("Sensor2 Detected",  detected2);
        telemetry.addData("Sensor2 Locked",    lastLockedColor2);
        telemetry.addData("Sensor1 Detected",  detected1);
        telemetry.addData("Sensor1 Locked",    lastLockedColor1);
        telemetry.addData("----- Shooter Data -----", null);
        double flywheel1Rpm = motorTicksPerSecondToFlywheelRpm(Math.abs(testmotor.getVelocity()));
        double flywheel2Rpm = motorTicksPerSecondToFlywheelRpm(Math.abs(flywheelmotor2.getVelocity()));
        telemetry.addData("AutoShoot: ",               autoShootMessage);
        telemetry.addData("Shooter Mode",              shooterModeName());
        telemetry.addData("Commanded Speed Percent",   commandedSpeedPercent);
        telemetry.addData("Target Flywheel RPM",       targetFlywheelRpm);
        telemetry.addData("Flywheel 1 RPM",            flywheel1Rpm);
        telemetry.addData("Flywheel 2 RPM",            flywheel2Rpm);
        telemetry.addData("Shooter Ready",             shooterReady());
        telemetry.addData("Motor Target deg/s",        flywheelRpmToMotorDegPerSec(targetFlywheelRpm));
        telemetry.addData("Motor Target ticks/s",      flywheelRpmToMotorTicksPerSecond(targetFlywheelRpm));
        telemetry.addData("Motor 1 actual ticks/s",    testmotor.getVelocity());
        telemetry.addData("Motor 2 actual ticks/s",    flywheelmotor2.getVelocity());
        telemetry.addData("Triple Shot Running",       tripleShotRunning);
        telemetry.addData("Triple Shot Index",         tripleShotIndex);
        telemetry.addData("Rapid Shot Spacing ms",     RAPID_FIRE_SHOT_SPACING_MS);
        telemetry.addData("Followup Speed Scale",      TRIPLE_SHOT_FOLLOWUP_SPEED_SCALE);
        telemetry.addData("testmotor current pos",     testmotor.getCurrentPosition());
        telemetry.addData("flywheel2 current pos",     flywheelmotor2.getCurrentPosition());
        telemetry.addData("----- Limelight Data -----", null);
        telemetry.addData("LimeLight(ty): ", limelightTy_local);
        telemetry.addData("LimeLight(tx): ", limelight_tx_local);
        telemetry.addData("Timer (s)",       timer.seconds());
        telemetry.update();
    }

    private double calculateFieldAdjustedSpeedPercent(double ty) {
        double speedPercent;
        if (ty >= CLOSE_SHOT_TY_THRESHOLD) {
            speedPercent = CLOSE_SHOT_SPEED_PERCENT;
        } else {
            speedPercent = FIELD_SPEED_A * ty + FIELD_SPEED_B + FIELD_SPEED_OFFSET;
        }

        return Math.max(0.0, Math.min(1.0, speedPercent));
    }

    private double applyTripleShotFollowupSpeedScale(double speedPercent) {
        if (tripleShotRunning && tripleShotIndex > 0) {
            return Math.max(0.0, Math.min(1.0, speedPercent * TRIPLE_SHOT_FOLLOWUP_SPEED_SCALE));
        }
        return speedPercent;
    }

    private String shooterModeName() {
        if (shooterMode == SHOOTER_MODE_IDLE) return "IDLE";
        if (shooterMode == SHOOTER_MODE_FIELD_ADJUSTED) return "FIELD_ADJUSTED";
        return "STOPPED";
    }

    private double flywheelRpmToMotorRpm(double flywheelRpm) {
        return flywheelRpm * MOTOR_REV_PER_FLYWHEEL_REV;
    }

    private double motorRpmToFlywheelRpm(double motorRpm) {
        return motorRpm / MOTOR_REV_PER_FLYWHEEL_REV;
    }

    private double flywheelRpmToMotorDegPerSec(double flywheelRpm) {
        double motorRpm = flywheelRpmToMotorRpm(flywheelRpm);
        return motorRpm * 360.0 / 60.0;
    }

    private double flywheelRpmToMotorTicksPerSecond(double flywheelRpm) {
        double motorRpm = flywheelRpmToMotorRpm(flywheelRpm);
        return motorRpm * MOTOR_TICKS_PER_REV / 60.0;
    }

    private double motorDegPerSecToFlywheelRpm(double motorDegPerSec) {
        double motorRpm = motorDegPerSec * 60.0 / 360.0;
        return motorRpmToFlywheelRpm(motorRpm);
    }

    private double motorTicksPerSecondToFlywheelRpm(double motorTicksPerSecond) {
        double motorRpm = motorTicksPerSecond * 60.0 / MOTOR_TICKS_PER_REV;
        return motorRpmToFlywheelRpm(motorRpm);
    }

    private void setShooterFlywheelRpm(double flywheelRpm) {
        targetFlywheelRpm = clipFlywheelRpmToMotorLimit(flywheelRpm);

        if (Math.abs(targetFlywheelRpm - lastAppliedTargetFlywheelRpm) > TARGET_CHANGE_RESET_RPM) {
            shooterWasReady = false;
            shooterReadyTimer.reset();
        }
        lastAppliedTargetFlywheelRpm = targetFlywheelRpm;

        double motorTicksPerSecond = flywheelRpmToMotorTicksPerSecond(targetFlywheelRpm);

        // Use raw encoder ticks/sec instead of AngleUnit.DEGREES so this does not depend
        // on the motor type selected in the Driver Station hardware configuration.
        testmotor.setVelocity(motorTicksPerSecond);
        flywheelmotor2.setVelocity(motorTicksPerSecond);
    }

    private double clipFlywheelRpmToMotorLimit(double flywheelRpm) {
        if (MOTOR_REV_PER_FLYWHEEL_REV <= 0) {
            return 0.0;
        }

        double maxFlywheelRpm = MOTOR_FREE_RPM / MOTOR_REV_PER_FLYWHEEL_REV;
        return Math.max(0.0, Math.min(flywheelRpm, maxFlywheelRpm));
    }

    private boolean shooterReady() {
        if (targetFlywheelRpm < 100.0) {
            shooterWasReady = false;
            shooterReadyTimer.reset();
            return false;
        }

        double motor1TicksPerSecond = Math.abs(testmotor.getVelocity());
        double motor2TicksPerSecond = Math.abs(flywheelmotor2.getVelocity());

        double flywheel1Rpm = motorTicksPerSecondToFlywheelRpm(motor1TicksPerSecond);
        double flywheel2Rpm = motorTicksPerSecondToFlywheelRpm(motor2TicksPerSecond);

        boolean motor1Ready = Math.abs(flywheel1Rpm - targetFlywheelRpm) < FLYWHEEL_READY_TOLERANCE_RPM;
        boolean motor2Ready = Math.abs(flywheel2Rpm - targetFlywheelRpm) < FLYWHEEL_READY_TOLERANCE_RPM;
        boolean readyNow = motor1Ready && motor2Ready;

        if (!readyNow) {
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

    private String detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double r = clamp01(rgba.red);
        double g = clamp01(rgba.green);
        double b = clamp01(rgba.blue);

        double intensity = r + g + b;
        boolean brightEnough = intensity > 0.06;
        boolean notBlownOut  = intensity < 2.7;

        double sum = Math.max(1e-6, intensity);
        double rn = r / sum;
        double gn = g / sum;
        double bn = b / sum;

        boolean isGreen  = brightEnough && notBlownOut && gn > 0.45 && gn > rn + 0.05 && gn > bn + 0.05;
        boolean isPurple = brightEnough && notBlownOut && rn > 0.20 && bn > 0.40 && gn < 0.35 && Math.abs(rn - bn) < 0.30;

        if (isGreen)  return "green";
        if (isPurple) return "purple";
        return "unknown";
    }

    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }
        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);
        booleanIncrementer++;
        return output;
    }
}