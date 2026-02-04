/*  *** TeleOp Robot Control Summary ***
    1. GamePad-1
        1.1. *Drive Train Related* code block - [ left_stick_y, left_stick_x ]  - direction
        1.2. *Drive Train Related* code block - [ y, a, b ]                     - speed
        1.3. *Drive Train Related* code block - [ left_trigger, right trigger ] - *Drive Train Related* code block speed
        1.4. *Jerk* code block                - [x]                             - shake
        o.1. *Do I see gangster rapper*       - [a]                             - start FlyWheel
        o.2. *Do I see gangster rapper*       - [b]                             - stop FlyWheel

    2. GamPad-2
        2.1. *Dpadformotor*   - [ dpad_left  ]                   - liftservo
        2.2. *Dpadformotor*   - [ dpad_right ]                   - lift2servo
        2.3. *Dpadformotor*   - [ dpad_down  ]                   - lift3servo
        2.4. *Intake*         - [ b, y, x    ]                   - Set Power
        2.5. *Dingus Shooter* - [ right_trigger ]                - 0.5 increment power
        2.6. *Dingus Shooter* - [ left_trigger ]                 - 0.5 decrement power
        2.7. *Dingus Shooter* - [ right_bumper ]                 - 0.70 preset power
        2.8. *Dingus Shooter* - [ left_bumper ]                  - 0.00 preset power
        2.9. *Do I see gangster rapper* - [a]                    - start & stop FlyWheel
   3. Auto
        3.1. *Do I see gangster rapper* - LLResult resultsofpooe - LimeLight Result
        3.2. *Do I see gangster rapper* - poopeemotorey - Turn Table motor
        3.2. *Do I see gangster rapper* - testmotor     - FlyWheel motor
        3.3. *Do I see gangster rapper* - hoodservo     - Hood Servo
 */
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.common.AutoShooter;

@TeleOp(name="Tuning AutoShootTeleop")
public class tuningAutoShootTeleop extends OpMode {

    DcMotor testmotor;
    DcMotor intakemotor;

    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;
    Servo liftservo;
    Servo lift2servo;
    Servo lift3servo;
    Servo hoodservo; //--- Added for AutoShoot

//    Servo aimservo;
//
//    Servo convey;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

    int cycleMode = 0; // 0 = none, 1 = lift1, 2 = lift2, 3 = lift3

    private boolean cycleRunning = false;
    private double cycleStartTime = 0;
    DcMotor poopeemotorey;
    private Limelight3A lookylookyseesee;
    ElapsedTime timer = new ElapsedTime();

    private boolean  rtWasPressed = false;
    private boolean ltWasPressed = false;

    double pPID = 0.04;
    double dPID = 0.001;
    double iPID = 0;
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

    //--- End AutoShoot

    double deadzone = 0.05;
    private boolean jerkRunning = false;
    private double jerkStartTime = 0;
    private ElapsedTime jerkTimer = new ElapsedTime();

    private double clampValue = 0.25;
    private double value = 0;
    public double speed = 1.0;
    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        testmotor = hardwareMap.dcMotor.get("testemotor");
        motor_frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        motor_frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        motor_backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        motor_backRight = hardwareMap.get(DcMotorEx.class, "rr");

        intakemotor = hardwareMap.dcMotor.get("intakemotor");

        poopeemotorey = hardwareMap.get(DcMotor.class, "turretmotor");
        //***** ShooterActuator Servo
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        //lookylookyseesee = hardwareMap.get(Limelight3A.class, "lookylookyseesee");
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
        //       aimservo = hardwareMap.get(Servo.class, "aimservo");
//        convey = hardwareMap.get(Servo.class, "convey");

        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftservo.setPosition(0.05); //0.05
        lift2servo.setPosition(0.745); //0.745
        lift3servo.setPosition(0.1);
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
        //boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        //boolean gamePad1bIsPressed = ifPressed(gamepad1.b);

        /***************************/
        /****Drive Train Related ***/
        /***************************/
        double vx = speed*(-gamepad1.left_stick_y * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double vy = speed*(gamepad1.left_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double o = speed*(gamepad1.right_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));//*//some value;

        if (gamePad1yIsPressed){
            speed = 0.7;
        }
        /*
        if (gamePad1bIsPressed){
            speed = 0.3;
        }
        if (gamePad1aIsPressed){
            speed = 1.5;
        }
        */
        double leftFrontPower  = vx + vy + o;
        double rightFrontPower = vx - vy - o;
        double leftBackPower   = vx - vy + o;
        double rightBackPower  = vx + vy - o;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

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

        //if (gamepad2.dpad_left  && !cycleRunning) { cycleRunning = true; cycleMode = 1; cycleStartTime = timer.milliseconds(); }
        //if (gamepad2.dpad_right && !cycleRunning) { cycleRunning = true; cycleMode = 2; cycleStartTime = timer.milliseconds(); }
        //if (gamepad2.dpad_down && !cycleRunning) { cycleRunning = true; cycleMode = 3; cycleStartTime = timer.milliseconds(); }
        if (gamepad1.dpad_left  && !cycleRunning) { cycleRunning = true; cycleMode = 1; cycleStartTime = timer.milliseconds(); }
        if (gamepad1.dpad_right && !cycleRunning) { cycleRunning = true; cycleMode = 2; cycleStartTime = timer.milliseconds(); }
        if (gamepad1.dpad_down && !cycleRunning) { cycleRunning = true; cycleMode = 3; cycleStartTime = timer.milliseconds(); }

        if (cycleRunning) {
            double t = timer.milliseconds() - cycleStartTime;

            if (cycleMode == 1) {
                if (t < 300) liftservo.setPosition(0.55); //0.55
                else if (t < 500) liftservo.setPosition(0.05); //0.05
                else { cycleRunning = false; cycleMode = 0; }
            }
            else if (cycleMode == 2) {
                if (t < 300) lift2servo.setPosition(0.30); //0.30
                else if (t < 500) lift2servo.setPosition(0.745);  //0.745
                else { cycleRunning = false; cycleMode = 0; }
            }
            else if (cycleMode == 3) {
                if (t < 300) lift3servo.setPosition(0.55);
                else if (t < 500) lift3servo.setPosition(0.1);
                else { cycleRunning = false; cycleMode = 0; }
            }
        }

        /***************************/
        /**** Intake ***/
        /***************************/

        if (gamepad2.b) {
            intakemotor.setPower(0);
        }
        if (gamepad2.y) {
            intakemotor.setPower(1.0);
        }
        if (gamepad2.x) {
            intakemotor.setPower(-1.0);
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

/*
// --- Incremental step controls (tap) ---
        double step = 0.05;

// Treat triggers like buttons (press past threshold)
        boolean rtPressed = gamepad2.right_trigger > 0.5;
        boolean ltPressed = gamepad2.left_trigger  > 0.5;

// Rising edge: step only once per press
        if (rtPressed && !rtWasPressed) {
            value += step;
        }
        if (ltPressed && !ltWasPressed) {
            value -= step;
        }

// Save states for next loop
        rtWasPressed = rtPressed;
        ltWasPressed = ltPressed;

// --- Presets OVERRIDE incremental ---
        if (gamepad2.left_bumper) {
            value = 0.70;
        } else if (gamepad2.right_bumper) {
            value = 0.0;
        }

// Clamp AFTER everything, then apply
        value = Math.max(0.0, Math.min(value, 1.0));
        testmotor.setPower(value);
*/

// Telemetry
///***************************/
///**** Do I see gangster rapper? ***/
///***************************/

        ///***************************/
        ///**** FlyWheel On/Off    ***/
        ///***************************/

        boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        boolean gamePad1bIsPressed = ifPressed(gamepad1.b);
        //Options GamePad1
        if (gamepad1.a && !gamePad1aIsPressed) {
            autoShoot.startShooter();
            autoShootMessage = "Pressed and started";
        }
        if (gamepad1.b && !gamePad1bIsPressed) {
            autoShoot.stopShooter();
            autoShootMessage = "Pressed and stopped";
        }


        //if (gamepad2.a && !aButtonWasPressed && !autoShoot.isShooterStopped()) {
        if (gamepad2.a && !aButtonWasPressed) {
            autoShoot.startShooter();
            autoShootMessage = "Pressed and started";
        }

        //if (gamepad2.b && !aButtonWasPressed && autoShoot.isShooterStopped()) {
        if (gamepad2.right_bumper && !bButtonWasPressed) {
            autoShoot.stopShooter();
            autoShootMessage = "Pressed and stopped";
        }
        aButtonWasPressed = gamepad2.a;
        bButtonWasPressed = gamepad2.right_bumper;

        ///*** End Flywheel On/Off ***/

        LLResult resultsofpooe = lookylookyseesee.getLatestResult();
        boolean doesiseeitfoundboi = false;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {


                //--- For Auto Aim ---//
//                Double TxValue = resultsofpooe.getTx();
                //--- Get LimeLight Tx
                limelight_tx = fr.getTargetXDegrees();
                limelightTy = fr.getTargetYDegrees();
                //--- If Red Target
                if (fr.getFiducialId() == 24) {
                    derivativeTx = 1000000000.0 * (limelight_tx - lastTx) / (System.nanoTime() - lastTimeUpdated);
                    poopeemotorey.setPower(pPID * limelight_tx + dPID * derivativeTx); //TxValue
                    doesiseeitfoundboi = true;
                    lastTx = limelight_tx;
                    lastTimeUpdated = System.nanoTime();
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

                    //--- Get LimeLight Ty
                    autoShoot.advancedMathematics(limelightTy);
                    //--- Shooter FlyWheel
                    shooterPowerValue = autoShoot.getFlywheelPower();  //--- Update Shooter FlyWheel math here, or use new shooter class for object
                    testmotor.setPower(shooterPowerValue);
                    //--- Shooter Angle
                    servoPositionValue = autoShoot.getAnglePosition();  //--- Update Shooter Angle math here, or use new shooter class for object
                    hoodservo.setPosition(servoPositionValue);
                    break;
                }

            }

        }
        if (!doesiseeitfoundboi) {
            poopeemotorey.setPower(0);
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

        // Telemetry feedback
        telemetry.addData( "Timer (s)", timer.seconds());
        //telemetry.addData("Shooter Power (0-1)", value);

        telemetry.addData("----- Shooter Data -----", null);
        telemetry.addData("AutoShoot: ", autoShootMessage);
        telemetry.addData("AutoShoot Stopped: ", autoShoot.isShooterStopped());
        telemetry.addData("AutoShoot(FlyWheel Power): ", shooterPowerValue);
        telemetry.addData("AutoShoot(Hood Position): ", servoPositionValue);
        telemetry.addData("AutoShoot(a Button): ", gamepad2.a);
        telemetry.addData("AutoShoot(a Button WasPressed): ", aButtonWasPressed);

        telemetry.addData("----- Limelight Data -----", null);
        telemetry.addData("Limelight: ", limelightMessage);
        telemetry.addData("LimeLight(ty): ", limelightTy);
        telemetry.addData("LimeLight(tx): ", limelight_tx);
        telemetry.update();
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
