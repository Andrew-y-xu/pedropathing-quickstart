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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.AutoShooter_UseVelocity;
import org.firstinspires.ftc.teamcode.util.IndicatorLight;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Blue Teleop - Set Velocity")
public class BlueTeleop_UseVelocity extends OpMode {

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

    double shooterValue = 0;
    double servoPositionValue = 0;
    static double TICKS_PER_REV = 28.0;
    static double MAX_RPM = 6000.0;
    static double SHOOTER_MIN_LIMIT = 0.0;
    static double SHOOTER_MAX_LIMIT = 5800.0;

    // FIX: persistent target velocity so it doesn't get overwritten each loop
    private double targetVelocity = 0.0;

    double servo_value = 0.5;
    String limelightMessage = "No data available";
    String autoShootMessage = "Shooter At Init";
    final AutoShooter_UseVelocity autoShoot = new AutoShooter_UseVelocity(
            TICKS_PER_REV,
            MAX_RPM,
            SHOOTER_MIN_LIMIT,
            SHOOTER_MAX_LIMIT
    );

    double deadzone = 0.05;
    private boolean jerkRunning = false;
    private double jerkStartTime = 0;
    private ElapsedTime jerkTimer = new ElapsedTime();

    private double value = 0;
    public double speed = 1.0;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static final String SENSOR3_NAME = "color3";
    private static final String SENSOR2_NAME = "color2";

    private static final String INDICATOR3_NAME = "indicator3";
    private static final String INDICATOR2_NAME = "indicator2";

    private NormalizedColorSensor colorSensor3;
    private NormalizedColorSensor colorSensor2;
    private double colorPauseEnd2 = 0;
    private double colorPauseEnd3 = 0;
    private boolean pauseColor2 = false;
    private boolean pauseColor3 = false;
    private IndicatorLight light3;
    private IndicatorLight light2;

    private String lastLockedColor3 = "unknown";
    private String lastLockedColor2 = "unknown";
    private int max_speed = 36000; // 6000 * 360 / 60

    @Override
    public void init() {
        testmotor      = hardwareMap.get(DcMotorEx.class, "testemotor");
        flywheelmotor2 = hardwareMap.get(DcMotorEx.class, "flywheelmotor2");
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

        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, SENSOR3_NAME);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, SENSOR2_NAME);

        light3 = new IndicatorLight(hardwareMap, INDICATOR3_NAME);
        light2 = new IndicatorLight(hardwareMap, INDICATOR2_NAME);

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
        if (gamepad2.dpad_left  && !cycleRunning) { cycleRunning = true; cycleMode = 1; cycleStartTime = timer.milliseconds(); }
        if (gamepad2.dpad_right && !cycleRunning) { cycleRunning = true; cycleMode = 2; cycleStartTime = timer.milliseconds(); }
        if (gamepad2.dpad_down  && !cycleRunning) { cycleRunning = true; cycleMode = 3; cycleStartTime = timer.milliseconds(); }

        if (cycleRunning) {
            double t = timer.milliseconds() - cycleStartTime;

            if (cycleMode == 1) {
                if      (t < 300) liftservo.setPosition(0.58);
                else if (t < 500) liftservo.setPosition(0.05);
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
        /**** Flywheel Velocity Control (FIXED) ***/
        /***************************/

        // A button: set to 15% speed (rising edge only)
        if (gamepad2.a && !aButtonWasPressed) {
            targetVelocity = max_speed * 0.15;
            autoShootMessage = "Set to 15%";
        }

        // Bumpers: override target velocity
        if (gamepad2.left_bumper) {
            targetVelocity = max_speed * 0.62;
            autoShootMessage = "Set to 62%";
        } else if (gamepad2.right_bumper) {
            targetVelocity = 0;
            autoShootMessage = "Stopped";
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

                if (fr.getFiducialId() == 20) {
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

        /***************************/
        /**** Apply Flywheel & Turret ***/
        /***************************/
        if (autoShoot.isShooterStopped() || !doesiseeitfoundboi) {

            // Manual turret control
            double turretPower = 0.0;
            if (gamepad1.left_trigger > 0.05) {
                turretPower =  gamepad1.left_trigger * 0.35;
            } else if (gamepad1.right_trigger > 0.05) {
                turretPower = -gamepad1.right_trigger * 0.35;
            }
            poopeemotorey.setPower(turretPower);

            // FIX: always apply the persistent targetVelocity every loop
            testmotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
            flywheelmotor2.setVelocity(targetVelocity, AngleUnit.DEGREES);

        } else {
            autoShoot.advancedMathematics(limelightTy_local);
            shooterValue       = autoShoot.getFlywheelValue();
            servoPositionValue = autoShoot.getAnglePosition();

            testmotor.setVelocity(max_speed * shooterValue, AngleUnit.DEGREES);
            flywheelmotor2.setVelocity(max_speed * shooterValue, AngleUnit.DEGREES);
            hoodservo.setPosition(servoPositionValue);
        }

        /***************************/
        /**** Color Sensors ***/
        /***************************/
        String detected3 = "unknown";
        String detected2 = "unknown";

        if (!pauseColor3 || timer.milliseconds() > colorPauseEnd3) {
            pauseColor3 = false;
            detected3 = detectColor(colorSensor3);
        }
        if (!pauseColor2 || timer.milliseconds() > colorPauseEnd2) {
            pauseColor2 = false;
            detected2 = detectColor(colorSensor2);
        }

        if (!pauseColor3 && (detected3.equals("green") || detected3.equals("purple"))) {
            lastLockedColor3 = detected3;
        }
        if (!pauseColor2 && (detected2.equals("green") || detected2.equals("purple"))) {
            lastLockedColor2 = detected2;
        }

        if      (pauseColor3)                         light3.white();
        else if (lastLockedColor3.equals("green"))    light3.green();
        else if (lastLockedColor3.equals("purple"))   light3.violet();

        if      (pauseColor2)                         light2.white();
        else if (lastLockedColor2.equals("green"))    light2.green();
        else if (lastLockedColor2.equals("purple"))   light2.violet();

        /***************************/
        /**** Telemetry ***/
        /***************************/
        telemetry.addData("Sensor3 Detected",  detected3);
        telemetry.addData("Sensor3 Locked",    lastLockedColor3);
        telemetry.addData("Sensor2 Detected",  detected2);
        telemetry.addData("Sensor2 Locked",    lastLockedColor2);
        telemetry.addData("----- Shooter Data -----", null);
        telemetry.addData("AutoShoot: ",               autoShootMessage);
        telemetry.addData("AutoShoot Stopped: ",       autoShoot.isShooterStopped());
        telemetry.addData("AutoShoot(FlyWheel Value):", shooterValue);
        telemetry.addData("AutoShoot(Hood Position):", servoPositionValue);
        telemetry.addData("Target Velocity (deg/s)",   targetVelocity);
        telemetry.addData("Flywheel Velocity",         testmotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("testmotor current pos",     testmotor.getCurrentPosition());
        telemetry.addData("flywheel2 current pos",     flywheelmotor2.getCurrentPosition());
        telemetry.addData("----- Limelight Data -----", null);
        telemetry.addData("LimeLight(ty): ", limelightTy_local);
        telemetry.addData("LimeLight(tx): ", limelight_tx_local);
        telemetry.addData("Timer (s)",       timer.seconds());
        telemetry.update();
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