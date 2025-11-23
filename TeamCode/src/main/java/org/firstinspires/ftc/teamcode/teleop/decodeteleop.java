package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@TeleOp(name="Decode Teleop M1")
public class decodeteleop extends OpMode {

    DcMotor testmotor;
    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;
    Servo liftservo;
    Servo convey;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

    private boolean cycleRunning = false;
    private double cycleStartTime = 0;

    ElapsedTime timer = new ElapsedTime();


    private boolean jerkRunning = false;
    private double jerkStartTime = 0;
    private ElapsedTime jerkTimer = new ElapsedTime();




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

        motor_frontLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_backRight.setDirection(DcMotor.Direction.FORWARD);
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


        liftservo = hardwareMap.get(Servo.class, "lift");
        convey = hardwareMap.get(Servo.class, "convey");

        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);




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
        /****Rest of the Shit***/
        /***************************/

        // Debounce input to prevent too frequent changes


        if (debounceTimer.milliseconds() > 100) {
            if (gamepad2.dpad_up) {
                value += 0.05;
                debounceTimer.reset();
            } else if (gamepad2.dpad_down) {
                value -= 0.05;
                debounceTimer.reset();
            }
        }

// Start the cycle when A is PRESSED (not held)
        if (gamepad2.a && !cycleRunning) {
            cycleRunning = true;
            cycleStartTime = timer.milliseconds();  // record start
        }

// Run the automatic up→down sequence
        if (cycleRunning) {
            double t = timer.milliseconds() - cycleStartTime;

            if (t < 200) {
                // FIRST 0–200 ms: move servo UP
                liftservo.setPosition(0.325);
            }
            else if (t < 400) {
                // NEXT 200–400 ms: move servo DOWN
                liftservo.setPosition(0.685);
            }
            else {
                // DONE — stop the cycle
                cycleRunning = false;
            }
        }


        // Clamp value to [0.0, 1.0]
        value = Math.max(0.0, Math.min(value, 1.0));

        //lift servo
        if (gamepad2.left_bumper) {
            value = 0.675;
        } else if (gamepad2.right_bumper) {
            value = 0;
        } else if (gamepad2.back) {
            value = -0.2;
        } else if (gamepad2.b) {
            value = 0.55;
        } else if (gamepad2.x) {
            value = 0.5;
        }
        testmotor.setPower(value);

        //conveyer belt
        if (gamepad2.y) {
            convey.setPosition(1.0);
        } else if (gamepad2.start) {
            convey.setPosition(0.0);
        } else {
            convey.setPosition(0.5);
        }

        // Telemetry feedback
        telemetry.addData("Servo Position", value);
        telemetry.addData( "Timer (s)", timer.seconds());
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


