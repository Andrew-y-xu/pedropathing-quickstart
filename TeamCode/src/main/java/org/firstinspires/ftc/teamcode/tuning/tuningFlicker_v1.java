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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AutoShooter;
import org.firstinspires.ftc.teamcode.common.AutoFlicker;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Tuning Flicker_v1")
public class tuningFlicker_v1 extends OpMode {

    //--- Common classes
    AutoFlicker autoFlicker;
    DcMotor testmotor;
    DcMotor intakemotor;
    Servo intakeservo;

    Servo liftservo;
    Servo lift2servo;
    Servo lift3servo;
    Servo hoodservo; //--- Added for AutoShoot

    boolean xAlrPressed = false;
    boolean yAlrPressed = false;

    double tuningShooterIncrementConstant = 0.05;
    double tuningServoIncrementConstant   = 0.05;

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

    ElapsedTime motorDebounce = new ElapsedTime();
    ElapsedTime servoDebounce = new ElapsedTime();

    double shooter_value=0.0;

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

        intakemotor = hardwareMap.dcMotor.get("intakemotor");
        intakeservo = hardwareMap.get(Servo.class, "intake2servo");

        poopeemotorey = hardwareMap.get(DcMotor.class, "turretmotor");
        //***** ShooterActuator Servo
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        lookylookyseesee = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        lookylookyseesee.pipelineSwitch(0);
        lookylookyseesee.start();

        liftservo = hardwareMap.get(Servo.class, "lift2");
        lift2servo = hardwareMap.get(Servo.class, "lift1");
        lift3servo = hardwareMap.get(Servo.class, "lift3");

        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);


        autoFlicker = new AutoFlicker(liftservo, lift2servo, lift3servo);
        //liftservo.setPosition(0.05); //0.05
        //lift2servo.setPosition(0.745); //0.745
        //lift3servo.setPosition(0.1);
        //       aimservo.setPosition(0.5);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        booleanIncrementer = 0;
        boolean gamePad1yIsPressed = ifPressed(gamepad1.y);
        //boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        //boolean gamePad1bIsPressed = ifPressed(gamepad1.b);

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
            intakeservo.setPosition(0);
        }
        if (gamepad2.y) {
            intakemotor.setPower(1.0);
            intakeservo.setPosition(1);
        }
        if (gamepad2.x) {
            intakemotor.setPower(-1.0);
            intakeservo.setPosition(-1);
        }

/***************************/
/**** Dingus Shooter (incremental + presets override) ***/
/***************************/


// Telemetry
///***************************/
///**** Do I see gangster rapper? ***/
///***************************/

        ///***************************/
        ///**** FlyWheel On/Off    ***/
        ///***************************/

        //***** MOTOR CONTROL (X / Y)
        if (motorDebounce.milliseconds() > 300) {
            if (gamepad1.x && !xAlrPressed) {
                shooter_value += tuningShooterIncrementConstant;
                autoShoot.setFlywheelPower(shooter_value);
                motorDebounce.reset();
            }

            if (gamepad1.y && !yAlrPressed) {
                shooter_value -= tuningShooterIncrementConstant;
                autoShoot.setFlywheelPower(shooter_value);
                motorDebounce.reset();
            }
        }
        xAlrPressed = gamepad1.x;
        yAlrPressed = gamepad1.y;

        //--- Stop the FlyWheel
        if (gamepad1.right_bumper) {
            shooter_value = 0.0;
            servo_value = 0.5;
        }
        testmotor.setPower(shooter_value);

        //***** SERVO CONTROL (A / B)
        if (servoDebounce.milliseconds() > 150) {
            if (gamepad1.a) {
                servo_value += tuningServoIncrementConstant;
                autoShoot.setAnglePosition(servo_value);
                servoDebounce.reset();
            } else if (gamepad1.b) {
                servo_value -= tuningServoIncrementConstant;
                autoShoot.setAnglePosition(servo_value);
                servoDebounce.reset();
            }
        }

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
                    break;
                }

            }

        }// End Vision condition

        //shooterPowerValue = autoShoot.getFlywheelPower();  //--- Update Shooter FlyWheel math here, or use new shooter class for object
        testmotor.setPower(shooterPowerValue);
        testmotor.setPower(0.50);
        //--- Shooter Angle
        servoPositionValue = autoShoot.getAnglePosition();  //--- Update Shooter Angle math here, or use new shooter class for object
        hoodservo.setPosition(servoPositionValue);

        if (!doesiseeitfoundboi) {
            poopeemotorey.setPower(0);
            limelightMessage = "No data available";
        } else {
            limelightMessage = "Data is available";
        }

        //***** MOTOR CONTROL (X / Y)
        if (motorDebounce.milliseconds() > 300) {
            if (gamepad1.x && !xAlrPressed) {
                shooterPowerValue += tuningShooterIncrementConstant;
                motorDebounce.reset();
            }

            if (gamepad1.y && !yAlrPressed) {
                shooterPowerValue -= tuningShooterIncrementConstant;
                motorDebounce.reset();
            }
        }
        xAlrPressed = gamepad1.x;
        yAlrPressed = gamepad1.y;

        //--- Stop the FlyWheel
        if (gamepad1.right_bumper) {
            shooterPowerValue = 0.0;
            servo_value = 0.5;
        }
        testmotor.setPower(shooterPowerValue);

        //***** SERVO CONTROL (A / B)
        if (servoDebounce.milliseconds() > 150) {
            if (gamepad1.a) {
                servoPositionValue += tuningServoIncrementConstant;
                servoDebounce.reset();
            } else if (gamepad1.b) {
                servoPositionValue -= tuningServoIncrementConstant;
                servoDebounce.reset();
            }
        }

        //***** Telemetry feedback
        servoPositionValue = Math.max(0.0, Math.min(servo_value, 1.0));
        hoodservo.setPosition(servoPositionValue);


        // Telemetry feedback
        telemetry.addData( "Timer (s)", timer.seconds());
        //telemetry.addData("Shooter Power (0-1)", value);

        telemetry.addData("----- Shooter Data -----", null);
        //telemetry.addData("AutoShoot: ", autoShootMessage);
        //telemetry.addData("AutoShoot Stopped: ", autoShoot.isShooterStopped());
        telemetry.addData("AutoShoot(FlyWheel Power): ", shooterPowerValue);
        telemetry.addData("AutoShoot(Hood Position): ", servoPositionValue);
        //telemetry.addData("AutoShoot(a Button): ", gamepad2.a);
        //telemetry.addData("AutoShoot(a Button WasPressed): ", aButtonWasPressed);

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