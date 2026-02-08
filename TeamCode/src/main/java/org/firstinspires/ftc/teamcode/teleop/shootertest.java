package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="Shooter Test", group="Test")
@Disabled
public class shootertest extends OpMode {

    DcMotorEx testmotor;


//    CRServo convey;
//    private Servo liftservo;

    private double value = 0;

    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        testmotor = hardwareMap.get(DcMotorEx.class, "testemotor");
        testmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        testmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        testmotor.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Debounce input to prevent too frequent changes
        if (debounceTimer.milliseconds() > 100) {
            if (gamepad1.x) {
                value += 25;
                debounceTimer.reset();
            } else if (gamepad1.y) {
                value -= 25;
                debounceTimer.reset();
            }
        }

//        if (gamepad1.a) {
//            liftservo.setPosition(0.325);
//        }
//        else {
//            liftservo.setPosition(0.685);
//        }
//
//        // Clamp value to [0.0, 1.0]
//        value = Math.max(0.0, Math.min(value, 1.0));

        // Reset to default position
        if (gamepad1.left_bumper) {
            value = 100;
        } else if (gamepad1.right_bumper) {
            value = 200;
        } else if (gamepad1.back) {
            value = 300;
        }

//        if (gamepad1.b) {
//            convey.setPower(0.5);
//        } else if (gamepad1.start) {
//            convey.setPower(-0.5);
//        } else {
//            convey.setPower(0.0);
//        }

        // Apply servo position
        testmotor.setVelocity(value);

        // Telemetry feedback
        telemetry.addData("Servo Position", value);
        telemetry.update();
    }
}