package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.hardware.AutoShooter;


import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.DetectableColor;
import org.firstinspires.ftc.teamcode.util.IndicatorLight;

@TeleOp(name="Red Teleop")
public class RedTeleop extends OpMode {

    DcMotor testmotor;
    DcMotor flywheelmotor2;
    DcMotor intakemotor;

    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;
    Servo liftservo;
    Servo lift2servo;
    Servo lift3servo;
    Servo intake2servo;
    Servo hoodservo; //--- Added for AutoShoot
    private DetectableColor pin0Color;
    private DetectableColor pin1Color;
    private DigitalChannel digitalPin0;
    private DigitalChannel digitalPin1;


//    Servo aimservo;
//
//    Servo convey;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

    int cycleMode = 0; // 0 = none, 1 = lift1, 2 = lift2, 3 = lift3

    private boolean cycleRunning = false;
    private double cycleStartTime = 0;
    private boolean fullCycleRunning = false;
    private double fullCycleStartTime = 0;
    DcMotor poopeemotorey;
    private Limelight3A lookylookyseesee;
    ElapsedTime timer = new ElapsedTime();

    private boolean rtWasPressed = false;
    private boolean ltWasPressed = false;


    //pid values
    double pPID = 0.015; //0.11 --> 0.04 (original value)
    double dPID = 0.003; //0.003 --> 0.001 (original value)
    double iPID = 0;
    double integralPID = 0;
    double lastTx = 0;
    double derivativeTx = 0;
    double lastTimeUpdated = 0;

    //--- AutoShoot
    final AutoShooter autoShoot = new AutoShooter();
    double limelight_tx = 0;
    double limelightTy = 0;
    private boolean aButtonWasPressed = false;
    private boolean bButtonWasPressed = false;
    double shooterPowerValue = 0;
    double servoPositionValue = 0;
    double servo_value = 0.5;
    String limelightMessage = "No data available";
    String autoShootMessage = "At Init";



    double deadzone = 0.05;
    private boolean jerkRunning = false;
    private double jerkStartTime = 0;
    private ElapsedTime jerkTimer = new ElapsedTime();

    private double clampValue = 0.25;
    private double value = 0;
    public double speed = 1.0;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static final String SENSOR3_NAME = "color3";
    private static final String SENSOR2_NAME = "color2";
    private static final String SENSOR1_NAME = "color1";

    private static final String INDICATOR3_NAME = "indicator3";
    private static final String INDICATOR2_NAME = "indicator2";
    private static final String INDICATOR1_NAME = "indicator1";
    Boolean holdup1=false;
    //left, lift 3
    Boolean holdup2=false;
    //right lift 1

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

    // Store last locked colors independently
    private String lastLockedColor3 = "unknown";
    private String lastLockedColor2 = "unknown";
    private String lastLockedColor1 = "unknown";
    double additionalpower=0;

    @Override
    public void init() {
        testmotor = hardwareMap.dcMotor.get("testemotor");
        flywheelmotor2 = hardwareMap.dcMotor.get("flywheelmotor2");

        motor_frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        motor_frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        motor_backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        motor_backRight = hardwareMap.get(DcMotorEx.class, "rr");

        intakemotor = hardwareMap.dcMotor.get("intakemotor");
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        poopeemotorey = hardwareMap.get(DcMotor.class, "turretmotor");
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

        //motor run mode
        motor_frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor_frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        liftservo = hardwareMap.get(Servo.class, "lift2");
        lift2servo = hardwareMap.get(Servo.class, "lift1");
        lift3servo = hardwareMap.get(Servo.class, "lift3");
        intake2servo = hardwareMap.get(Servo.class, "intake2servo");
        //       aimservo = hardwareMap.get(Servo.class, "aimservo");
//        convey = hardwareMap.get(Servo.class, "convey");

        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelmotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, SENSOR3_NAME);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, SENSOR2_NAME);
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, SENSOR1_NAME);

        light3 = new IndicatorLight(hardwareMap, INDICATOR3_NAME);
        light2 = new IndicatorLight(hardwareMap, INDICATOR2_NAME);
        light1 = new IndicatorLight(hardwareMap, INDICATOR1_NAME);
        liftservo.setPosition(0.05); //0.05
        lift2servo.setPosition(0.98); //0.9
        lift3servo.setPosition(0.12);
        //       aimservo.setPosition(0.5);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void frontLeft(double power){
        if (power > 1){
            power = 1;
        }
        motor_frontLeft.setPower(power);
    }
    public void frontRight(double power){
        if (power > 1){
            power = 1;
        }
        motor_frontRight.setPower(power);
    }
    public void backLeft(double power){
        if (power > 1){
            power = 1;
        }
        motor_backLeft.setPower(power);
    }
    public void backRight(double power){
        if (power > 1){
            power = 1;
        }
        motor_backRight.setPower(power);
    }

    @Override
    public void loop() {
        booleanIncrementer = 0;
        boolean gamePad1yIsPressed = ifPressed(gamepad1.y);
        boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        boolean gamePad1bIsPressed = ifPressed(gamepad1.b);

        /***************************/
        /****Drive Train Related ***/
        /***************************/
        double vx = speed*(-gamepad1.left_stick_y * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double vy = speed*(gamepad1.left_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double o = speed*(gamepad1.right_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));//*//some value;

        if (gamePad1yIsPressed){
            speed = 0.7;
        }
        if (gamePad1bIsPressed){
            speed = 0.3;
        }
        if (gamePad1aIsPressed){
            speed = 1.5;
        }

        double leftFrontPower  = vx + vy + o;
        double rightFrontPower = vx - vy - o;
        double leftBackPower   = vx - vy + o;
        double rightBackPower  = vx + vy - o;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = max(max, Math.abs(leftBackPower));
        max = max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // calculate the speed for each wheel and drive
        frontLeft(leftFrontPower);
        frontRight(rightFrontPower);
        backLeft(leftBackPower);
        backRight(rightBackPower);

        telemetry.addData("frontLeft", leftFrontPower);
        telemetry.addData("frontRight", rightFrontPower);
        telemetry.addData("backLeft", leftBackPower);
        telemetry.addData("backRight", rightBackPower);



        /***************************/
        /****Jerk ***/
        /***************************/
        // Start jerk sequence on button press
        if (gamepad1.x && !jerkRunning) {
            jerkRunning = true;
            jerkStartTime = jerkTimer.milliseconds();
        }

// Run the jerk sequence
        if (jerkRunning) {
            double t = jerkTimer.milliseconds() - jerkStartTime;

            if (t < 210) {
                // Drive forward fast for 200 ms
                frontLeft(1.0);
                frontRight(1.0);
                backLeft(1.0);
                backRight(1.0);
            }
            else if (t < 400) {
                // Then drive backwards fast for 200 ms
                frontLeft(-1.0);
                frontRight(-1.0);
                backLeft(-1.0);
                backRight(-1.0);
            }
            else if (t < 420) {
                frontLeft(0.9);
                backLeft(0.9);
                frontRight(-0.9);
                backRight(-0.9);
            }
            else {
                // End sequence
                jerkRunning = false;

                // STOP the robot
                frontLeft(0);
                frontRight(0);
                backLeft(0);
                backRight(0);
            }
        }


        /***************************/
        /**** Dpadformotor? ***/
        /***************************/
//
//        if (debounceTimer.milliseconds() > 100) {
//            if (gamepad2.dpad_up) {
//                value += 0.05;
//                debounceTimer.reset();
//            } else if (gamepad2.dpad_down) {
//                value -= 0.05;
//                debounceTimer.reset();
//            }
//        }
        if(gamepad2.left_trigger>0.5){
            holdup1=true;
            liftservo.setPosition(1);
        }
        else{
            holdup1=false;
        }
        if(gamepad2.right_trigger>0.5){
            holdup2=true;
            lift2servo.setPosition(0);
        }
        else{
            holdup2=false;
        }
        /* Triple flicker action */
        if (gamepad2.dpad_up && !fullCycleRunning && !holdup1 &&!holdup2) { fullCycleRunning = true; fullCycleStartTime = timer.milliseconds(); }

        if (fullCycleRunning) {
            double t = timer.milliseconds() - fullCycleStartTime;

            /* Right */
            if (t < 300) lift2servo.setPosition(0.47); //0.30
            else if (t < 500) {
                lift2servo.setPosition(0.98);
                lastLockedColor2 = "unknown";
                light2.white();
                pauseColor2 = true;
                colorPauseEnd2 = timer.milliseconds() + 2500;
            }
            /* Left */
            else if (t < 800) liftservo.setPosition(0.58); //0.55
            else if (t < 1000) {
                liftservo.setPosition(0.05);//0.05
                lastLockedColor1 = "unknown";
                light1.white();
                pauseColor1 = true;
                colorPauseEnd1 = timer.milliseconds() + 2500;
            }
            /* Back */
            else if (t < 1300) lift3servo.setPosition(0.63);
            else if (t < 1500) {
                lift3servo.setPosition(0.12);
                lastLockedColor3 = "unknown";
                light3.white();
                pauseColor3 = true;
                colorPauseEnd3 = timer.milliseconds() + 2500;
            }
            else { fullCycleRunning = false; }
        }



        if (gamepad2.dpad_left  && !cycleRunning && !holdup1) { cycleRunning = true; cycleMode = 1; cycleStartTime = timer.milliseconds(); }
        if (gamepad2.dpad_right && !cycleRunning && !holdup2) { cycleRunning = true; cycleMode = 2; cycleStartTime = timer.milliseconds(); }
        if (gamepad2.dpad_down && !cycleRunning) { cycleRunning = true; cycleMode = 3; cycleStartTime = timer.milliseconds(); }

        if (cycleRunning) {
            double t = timer.milliseconds() - cycleStartTime;


            //slot1 = lift2servo, slot2 = liftservo, slot3 = lift3servo



            if (cycleMode == 1) {
                if (t < 300) liftservo.setPosition(0.58); //0.55
                else if (t < 500) {
                    liftservo.setPosition(0.05);//0.05
                    lastLockedColor1 = "unknown";
                    light1.white();
                    pauseColor1 = true;
                    colorPauseEnd1 = timer.milliseconds() + 2500;
                }
                else { cycleRunning = false; cycleMode = 0; }
            }
            else if (cycleMode == 2) {
                if (t < 300) lift2servo.setPosition(0.47); //0.30
                else if (t < 500) {
                    lift2servo.setPosition(0.98);
                    lastLockedColor2 = "unknown";
                    light2.white();
                    pauseColor2 = true;
                    colorPauseEnd2 = timer.milliseconds() + 2500;
                }
                else { cycleRunning = false; cycleMode = 0; }
            }
            else if (cycleMode == 3) {
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



///***************************/
///**** Clamp Servo – gamepad2 RIGHT stick smooth control ***/
///***************************/
//
//// FTC Y stick is inverted
//        double rightStickInput = -gamepad2.right_stick_y;
//
//// Deadzone check
//        if (Math.abs(rightStickInput) > deadzone) {
//            double servoRampRate = 0.005; // smaller than motor for precision
//            clampValue += rightStickInput * servoRampRate;
//        }
//
//// Clamp servo range to [0.25, 0.5]
//        clampValue = Math.max(0.25, Math.min(clampValue, 0.5));
//
//// Apply to servo
//        aimservo.setPosition(clampValue);


        // Clamp value to [0.0, 1.0]

/***************************/
/**** Dingus Shooter (incremental + presets override) ***/
/***************************/
//
//// --- Incremental step controls (tap) ---
//        double step = 0.05;
//
//// Treat triggers like buttons (press past threshold)
//        boolean rtPressed = gamepad2.right_trigger > 0.5;
//        boolean ltPressed = gamepad2.left_trigger  > 0.5;
//
//// Rising edge: step only once per press
//        if (rtPressed && !rtWasPressed) {
//            value += step;
//        }
//        if (ltPressed && !ltWasPressed) {
//            value -= step;
//        }
//
//// Save states for next loop
//        rtWasPressed = rtPressed;
//        ltWasPressed = ltPressed;
//
//// --- Presets OVERRIDE incremental ---
//        if (gamepad2.left_bumper) {
//            value = 0.70;
//        } else if (gamepad2.right_bumper) {
//            value = 0.0;
//        }

// Clamp AFTER everything, then apply
        value = max(0.0, min(value, 1.0));


// Telemetry
/***************************/
/**** Do I see gangster rapper? ***/
/***************************/


        //if (gamepad2.a && !aButtonWasPressed && !autoShoot.isShooterStopped()) {
        if (gamepad2.a && !aButtonWasPressed && autoShoot.isShooterStopped()) {
            autoShoot.startShooter();
            autoShootMessage = "Pressed and started";
        }

        //if (gamepad2.b && !aButtonWasPressed && autoShoot.isShooterStopped()) {
        if (gamepad2.right_bumper && !bButtonWasPressed && !autoShoot.isShooterStopped()) {
            autoShoot.stopShooter();
            autoShootMessage = "Pressed and stopped";
        }
        aButtonWasPressed = gamepad2.a;
        bButtonWasPressed = gamepad2.right_bumper;

        ///*** End Flywheel On/Off ***/

        LLResult resultsofpooe = lookylookyseesee.getLatestResult();
        boolean doesiseeitfoundboi = false;
        double dt = 0;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {

                telemetry.addData("FiducialID", fr.getFiducialId());
                //--- For Auto Aim ---//
//                Double TxValue = resultsofpooe.getTx();
                //--- Get LimeLight Tx
                limelight_tx = fr.getTargetXDegrees();
                limelightTy = fr.getTargetYDegrees();
                //--- If Red Target
                if (fr.getFiducialId() == 24) {
                    dt = System.nanoTime() - lastTimeUpdated;
                    derivativeTx = 1000000000.0 * (limelight_tx - lastTx) / (dt);
                    integralPID += limelight_tx * dt/1000000000;
                    poopeemotorey.setPower(pPID * limelight_tx + dPID * derivativeTx + iPID * integralPID); //TxValue
                    doesiseeitfoundboi = true;
                    lastTx = limelight_tx;
                    lastTimeUpdated = System.nanoTime();



                    //--- Get LimeLight Ty
//                    autoShoot.advancedMathematics(limelightTy);
//                    //--- Shooter FlyWheel
//                    shooterPowerValue = autoShoot.getFlywheelPower();  //--- Update Shooter FlyWheel math here, or use new shooter class for object
//                    testmotor.setPower(shooterPowerValue);
//                    flywheelmotor2.setPower(shooterPowerValue);
//                    //--- Shooter Angle
//                    servoPositionValue = autoShoot.getAnglePosition();  //--- Update Shooter Angle math here, or use new shooter class for object
//                    hoodservo.setPosition(servoPositionValue);
                    break;
                    //break;
                    //}

                    //--- For Auto Shoot (Auto Shoot depends on Limelight finding the target for distant ---//
                    //--- Future FlyWheel & Angle On/Off
                /*
                if (gamepad2.left_bumper) {
                    //--- Get LimeLight Ty
                    double ty = fr.getTargetYDegrees();
                    //--- Shooter FlyWheel
                    shooterPowerValue = autoShoot.getFlywheelPower;
                    testmotor.setPower(shooterPowerValue);

                    //--- Shooter Angle
                    servoPositionValue = autoShoot.getAnglePosition;
                    hoodservo.setPosition(servoPositionValue);
                }
                */

//                    //--- Get LimeLight Ty
//                    autoShoot.advancedMathematics(limelightTy);
//                    //--- Shooter FlyWheel
//                    shooterPowerValue = autoShoot.getFlywheelPower();  //--- Update Shooter FlyWheel math here, or use new shooter class for object
//                    testmotor.setPower(shooterPowerValue);
//                    //--- Shooter Angle
//                    servoPositionValue = autoShoot.getAnglePosition();  //--- Update Shooter Angle math here, or use new shooter class for object
//                    hoodservo.setPosition(servoPositionValue);
//                    break;


                }

            }

        }
        if (!doesiseeitfoundboi) {
            limelightMessage = "No data available";
        } else {
            limelightMessage = "Data is available";
        }
        //telemetry.update();


//
//        else if (gamepad2.back) {
//            value = -0.2;
//        } else if (gamepad2.b) {
//            value = 0.55;
//        } else if (gamepad2.x) {
//            value = 0.5;
//        }


//        //conveyer belt
//        if (gamepad2.y) {
//            convey.setPosition(1.0);
//        } else if (gamepad2.start) {
//            convey.setPosition(0.0);
//        } else {
//            convey.setPosition(0.5);
//        }

        //CHangeing

        if (autoShoot.isShooterStopped() || !doesiseeitfoundboi) {

            if (gamepad2.left_bumper) {
                testmotor.setPower(0.62);
                flywheelmotor2.setPower(0.62);
            }
            if (gamepad2.right_bumper) {
                testmotor.setPower(0.3);
                flywheelmotor2.setPower(0.3);
            }

            double turretPower = 0.0; // DEFAULT = stop

            if (gamepad1.left_trigger > 0.05) {
                turretPower =  gamepad1.left_trigger * 0.65;
            }
            else if (gamepad1.right_trigger > 0.05) {
                turretPower = -gamepad1.right_trigger * 0.65;
            }

            poopeemotorey.setPower(turretPower); // ALWAYS set

        } else {
            autoShoot.advancedMathematics(limelightTy);
            shooterPowerValue = autoShoot.getFlywheelPower();
            servoPositionValue = autoShoot.getAnglePosition();
            testmotor.setPower(max(0,min(1,shooterPowerValue+additionalpower)));
            flywheelmotor2.setPower(max(0,min(1,shooterPowerValue+additionalpower)));
            hoodservo.setPosition(servoPositionValue);
        }
        if(gamepad1.back||gamepad2.back){
            additionalpower-=0.01;
        }
        if(gamepad1.start||gamepad2.start){
            additionalpower+=0.01;
        }

        //Change
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

        /* ADD THIS PART BACK */

// Sensor 3 locking
        if (!pauseColor3) {
            if (detected3.equals("green") || detected3.equals("purple")) {
                lastLockedColor3 = detected3;
            }
        }

// Sensor 2 locking
        if (!pauseColor2) {
            if (detected2.equals("green") || detected2.equals("purple")) {
                lastLockedColor2 = detected2;
            }
        }

        if(!pauseColor1){
            if(detected1.equals("green") || detected1.equals("purple")){
                lastLockedColor1 = detected1;
            }
        }
        if (pauseColor3) {
            light3.white();
        }
        else if (lastLockedColor3.equals("green")) {
            light3.green();
        }
        else if (lastLockedColor3.equals("purple")) {
            light3.violet();
        }
        if (pauseColor2) {
            light2.white();
        }
        else if (lastLockedColor2.equals("green")) {
            light2.green();
        }
        else if (lastLockedColor2.equals("purple")) {
            light2.violet();
        }
        if(pauseColor1){
            light1.white();
        }
        else if(lastLockedColor1.equals("green")){
            light1.green();
        }
        else if(lastLockedColor1.equals("purple")){
            light1.violet();
        }
        telemetry.addData("Sensor3 Detected", detected3);
        telemetry.addData("Sensor3 Locked", lastLockedColor3);
        telemetry.addData("Sensor2 Detected", detected2);
        telemetry.addData("Sensor2 Locked", lastLockedColor2);
        telemetry.addData("----- Shooter Data -----", null);
        telemetry.addData("AutoShoot: ", autoShootMessage);
        telemetry.addData("AutoShoot Stopped: ", autoShoot.isShooterStopped());
        telemetry.addData("AutoShoot(FlyWheel Power): ", shooterPowerValue);
        telemetry.addData("AutoShoot(Hood Position): ", servoPositionValue);
        telemetry.addData("AutoShoot(a Button): ", gamepad2.a);
        telemetry.addData("AutoShoot(a Button WasPressed): ", aButtonWasPressed);
        telemetry.addData("Additional Flywheel Power",additionalpower);
        telemetry.addData("Holding Left Up",holdup1);
        telemetry.addData("Holding Right Up",holdup2);


        telemetry.addData("----- Limelight Data -----", null);
        telemetry.addData("Limelight: ", limelightMessage);
        telemetry.addData("Limelight Time: ", (System.nanoTime() - lastTimeUpdated)/1000000.0);
        telemetry.addData("LimeLight(ty): ", limelightTy);
        telemetry.addData("LimeLight(tx): ", limelight_tx);
        telemetry.update();
        // Telemetry feedback
        telemetry.addData( "Timer (s)", timer.seconds());
        telemetry.addData("Shooter Power (0-1)", value);
        telemetry.update();
    }
    private String detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double r = clamp01(rgba.red);
        double g = clamp01(rgba.green);
        double b = clamp01(rgba.blue);

        double intensity = r + g + b;
        boolean brightEnough = intensity > 0.06;
        boolean notBlownOut = intensity < 2.7;

        double sum = max(1e-6, intensity);
        double rn = r / sum;
        double gn = g / sum;
        double bn = b / sum;

        boolean isGreen = brightEnough && notBlownOut &&
                gn > 0.45 &&
                gn > rn + 0.05 &&
                gn > bn + 0.05;

        boolean isPurple = brightEnough && notBlownOut &&
                rn > 0.20 &&
                bn > 0.40 &&
                gn < 0.35 &&
                Math.abs(rn - bn) < 0.30;

        if (isGreen) return "green";
        if (isPurple) return "purple";
        return "unknown"; // does NOT reset lock
    }

    public String isColorDetected(DetectableColor color) {
        requireDigital();
        if (color == pin0Color && digitalPin0.getState()) {
            return "purple";
        }
        if (color == pin1Color && digitalPin1.getState()) {
            return "green";
        }
        return "unknown";
    }
    private void requireDigital() {
        if (digitalPin0 == null || digitalPin1 == null) {
            throw new IllegalStateException(
                    "ColorDetector was not created with "
                            + "forDigitalRead(). Cannot read digital "
                            + "pins."
            );
        }
    }
    private double clamp01(float v) {
        return max(0.0, min(1.0, v));
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
        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }

}