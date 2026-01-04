package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.teleop.colorsensoring;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
@Disabled
@TeleOp(name="Decode Teleop M1")
public class decodeteleop extends OpMode {

    DcMotor testmotor;
    DcMotor intakemotor;
//    NormalizedColorSensor didisensor;
//    NormalizedColorSensor didi2sensor;
//    NormalizedColorSensor didi3sensor;

    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;
    Servo liftservo;
    Servo lift2servo;
    Servo lift3servo;
    // 1 = GREEN request, 2 = PURPLE request

//    Servo aimservo;
//
//    Servo convey;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;
    DcMotor poopeemotorey;
    private Limelight3A lookylookyseesee;
    ElapsedTime timer = new ElapsedTime();

    private boolean rtWasPressed = false;
    private boolean ltWasPressed = false;

    // Queue of WHICH servo/bin to actuate for each color
    private java.util.ArrayDeque<Integer> greenQueue = new java.util.ArrayDeque<>();
    private java.util.ArrayDeque<Integer> purpleQueue = new java.util.ArrayDeque<>();

    // Prevent duplicates if the same object is sitting under the sensor
    private boolean[] greenLatched = new boolean[4];  // index 1..3
    private boolean[] purpleLatched = new boolean[4];

    // Cycle runner state (only one cycle at a time, like your old code)
    private boolean cycleRunning = false;
    private int activeBin = 0;          // 1..3 (which servo set to run)
    private int activeColor = 0;        // 1 = green cycle, 2 = purple cycle
    private double cycleStartTime = 0;

    // Button edge detect
    private boolean prevGreenBtn = false;
    private boolean prevPurpleBtn = false;

    // Tune: how many stable frames before we count it as “detected”
    private int[] gStreak = new int[4];
    private int[] pStreak = new int[4];
    private static final int STREAK_TO_COUNT = 5;

    private void registerToken(int bin, boolean isGreenNow, boolean isPurpleNow) {

        // debounce streaking
        gStreak[bin] = isGreenNow ? gStreak[bin] + 1 : 0;
        pStreak[bin] = isPurpleNow ? pStreak[bin] + 1 : 0;

        boolean greenStable = gStreak[bin] >= STREAK_TO_COUNT;
        boolean purpleStable = pStreak[bin] >= STREAK_TO_COUNT;

        // Green edge detect: stable true AND not already latched
        if (greenStable && !greenLatched[bin]) {
            greenQueue.addLast(bin);
            greenLatched[bin] = true;
        }
        if (!greenStable) {
            greenLatched[bin] = false;
        }

        // Purple edge detect
        if (purpleStable && !purpleLatched[bin]) {
            purpleQueue.addLast(bin);
            purpleLatched[bin] = true;
        }
        if (!purpleStable) {
            purpleLatched[bin] = false;
        }
    }

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

        poopeemotorey = hardwareMap.get(DcMotor.class, "poopeemotorey");
        lookylookyseesee = hardwareMap.get(Limelight3A.class, "lookylookyseesee");
//        didisensor = hardwareMap.get(NormalizedColorSensor.class, "didisensor");
//        didi2sensor = hardwareMap.get(NormalizedColorSensor.class, "didi2sensor");
//        didi3sensor = hardwareMap.get(NormalizedColorSensor.class, "didi3sensor");

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
    private void doGreenServoThing() {

    }

    private void doPurpleServoThing() {

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
    private void startCycle(int bin, int color) {
        cycleRunning = true;
        activeBin = bin;
        activeColor = color;
        cycleStartTime = timer.milliseconds();
    }
    private void runCycle() {
        double t = timer.milliseconds() - cycleStartTime;

        // Example: Use SAME timing (0-300ms first pos, 300-500ms second pos)
        // but DIFFERENT positions per bin AND per color.
        // You can make green and purple do different motions if needed.

        if (activeColor == 1) { // GREEN cycle
            if (activeBin == 1) {
                if (t < 300) liftservo.setPosition(0.55);
                else if (t < 500) liftservo.setPosition(0.05);
                else stopCycle();
            } else if (activeBin == 2) {
                if (t < 300) lift2servo.setPosition(0.30);
                else if (t < 500) lift2servo.setPosition(0.745);
                else stopCycle();
            } else if (activeBin == 3) {
                if (t < 300) lift3servo.setPosition(0.55);
                else if (t < 500) lift3servo.setPosition(0.10);
                else stopCycle();
            }
        }

        else if (activeColor == 2) { // PURPLE cycle
            // If purple uses the SAME servo motions as green, you can copy the same blocks.
            // If purple should do different positions, put those here.

            if (activeBin == 1) {
                if (t < 300) liftservo.setPosition(0.55);
                else if (t < 500) liftservo.setPosition(0.05);
                else stopCycle();
            } else if (activeBin == 2) {
                if (t < 300) lift2servo.setPosition(0.30);
                else if (t < 500) lift2servo.setPosition(0.745);
                else stopCycle();
            } else if (activeBin == 3) {
                if (t < 300) lift3servo.setPosition(0.55);
                else if (t < 500) lift3servo.setPosition(0.10);
                else stopCycle();
            }
        }
    }

    private void stopCycle() {
        cycleRunning = false;
        activeBin = 0;
        activeColor = 0;
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

//        if (gamepad2.dpad_left  && !cycleRunning) { cycleRunning = true; cycleMode = 1; cycleStartTime = timer.milliseconds(); }
//        if (gamepad2.dpad_right && !cycleRunning) { cycleRunning = true; cycleMode = 2; cycleStartTime = timer.milliseconds(); }
//        if (gamepad2.dpad_down && !cycleRunning) { cycleRunning = true; cycleMode = 3; cycleStartTime = timer.milliseconds(); }
//
//        if (cycleRunning) {
//            double t = timer.milliseconds() - cycleStartTime;
//
//            if (cycleMode == 1) {
//                if (t < 300) liftservo.setPosition(0.55); //0.55
//                else if (t < 500) liftservo.setPosition(0.05); //0.05
//                else { cycleRunning = false; cycleMode = 0; }
//            }
//            else if (cycleMode == 2) {
//                if (t < 300) lift2servo.setPosition(0.30); //0.30
//                else if (t < 500) lift2servo.setPosition(0.745);  //0.745
//                else { cycleRunning = false; cycleMode = 0; }
//            }
//            else if (cycleMode == 3) {
//                if (t < 300) lift3servo.setPosition(0.55);
//                else if (t < 500) lift3servo.setPosition(0.1);
//                else { cycleRunning = false; cycleMode = 0; }
//            }
//        }

        boolean greenPressed  = gamepad2.dpad_left  && !prevGreenBtn;
        boolean purplePressed = gamepad2.dpad_right && !prevPurpleBtn;

        prevGreenBtn  = gamepad2.dpad_left;
        prevPurpleBtn = gamepad2.dpad_right;

        if (greenPressed && !cycleRunning && !greenQueue.isEmpty()) {
            int bin = greenQueue.removeFirst();
            startCycle(bin, 1);
        } else if (greenPressed && greenQueue.isEmpty()) {
            telemetry.addLine("No GREEN detected.");
        }

        if (purplePressed && !cycleRunning && !purpleQueue.isEmpty()) {
            int bin = purpleQueue.removeFirst();
            startCycle(bin, 2);
        } else if (purplePressed && purpleQueue.isEmpty()) {
            telemetry.addLine("No PURPLE detected.");
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

// Telemetry
/***************************/
/**** Do I see gangster rapper? ***/
/***************************/

        LLResult resultsofpooe = lookylookyseesee.getLatestResult();
        boolean doesiseeitfoundboi = false;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
//                Double TxValue = resultsofpooe.getTx();
                double tx = fr.getTargetXDegrees();
                if(fr.getFiducialId()==24) {
                    poopeemotorey.setPower(-0.018 * tx); //TxValue
                    doesiseeitfoundboi = true;
                    break;
                }
            }

        }
        if (!doesiseeitfoundboi) {
            poopeemotorey.setPower(0);
            telemetry.addData("Limelight", "No data available");
        }





/***************************/
/**** six seven loves color sensoring ***/
/***************************/


//        NormalizedRGBA rgba1 = didisensor.getNormalizedColors();
//        NormalizedRGBA rgba2 = didi2sensor.getNormalizedColors();
//        NormalizedRGBA rgba3 = didi3sensor.getNormalizedColors();
//
//        colorsensoring.Result sensor1Result = colorsensoring.classify(rgba1);
//        colorsensoring.Result sensor2Result = colorsensoring.classify(rgba2);
//        colorsensoring.Result sensor3Result = colorsensoring.classify(rgba3);
//
//        registerToken(1, sensor1Result.isGreen, sensor1Result.isPurple);
//        registerToken(2, sensor2Result.isGreen, sensor2Result.isPurple);
//        registerToken(3, sensor3Result.isGreen, sensor3Result.isPurple);
//
//
//
//        telemetry.addData("G queue", greenQueue.toString());
//        telemetry.addData("P queue", purpleQueue.toString());


/***************************/
/**** six seven loves color sensoring logic ***/
/***************************/


//
//        else if (gamepad2.back) {
//            value = -0.2;
//        } else if (gamepad2.b) {
//            value = 0.55;
//        } else if (gamepad2.x) {
//            value = 0.5;
//        }
        if (cycleRunning) {
            runCycle();
        }


//        //conveyer belt
//        if (gamepad2.y) {
//            convey.setPosition(1.0);
//        } else if (gamepad2.start) {
//            convey.setPosition(0.0);
//        } else {
//            convey.setPosition(0.5);
//        }

        // Telemetry feedback
        telemetry.addData("GreenQ", greenQueue.toString());
        telemetry.addData("PurpleQ", purpleQueue.toString());

//        telemetry.addData("S1", "G:%b P:%b", sensor1Result.isGreen, sensor1Result.isPurple);
//        telemetry.addData("S2", "G:%b P:%b", sensor2Result.isGreen, sensor2Result.isPurple);
//        telemetry.addData("S3", "G:%b P:%b", sensor3Result.isGreen, sensor3Result.isPurple);
        telemetry.addData( "Timer (s)", timer.seconds());
        telemetry.addData("Shooter Power (0-1)", value);
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
