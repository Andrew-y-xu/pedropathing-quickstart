package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

@TeleOp(name="Servo Test", group="Test")
public class servotest extends OpMode {

    private Servo liftservo;
    private double value = 0.5;

    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        liftservo = hardwareMap.get(Servo.class, "hood");

        liftservo.setPosition(value);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Debounce input to prevent too frequent changes
        if (debounceTimer.milliseconds() > 100) {
            if (gamepad1.x) {
                value += 0.02;
                debounceTimer.reset();
            } else if (gamepad1.y) {
                value -= 0.02;
                debounceTimer.reset();
            }
        }

        // Clamp value to [0.0, 1.0]
        value = Math.max(0.0, Math.min(value, 1.0));

        // Reset to default position
        if (gamepad1.left_bumper) {
            value = 1;
        } else if (gamepad1.right_bumper) {
            value = 0;
        } else if (gamepad1.back) {
            value = 0.5;
        }

        // Apply servo position
        liftservo.setPosition(value);

        // Telemetry feedback
        telemetry.addData("Servo Position", value);
        telemetry.update();
    }
}
