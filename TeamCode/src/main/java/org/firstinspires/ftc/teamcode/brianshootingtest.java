package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Brian Shooting Test")
@Disabled
public class brianshootingtest extends OpMode {

    // Shooter motor
    DcMotor shooterMotor;
    double shooterPower = 0;
    boolean rtWasPressed = false;
    boolean ltWasPressed = false;
    double step = 0.10; // incremental step for manual tuning

    // Optional: flicker servos
    Servo flicker1, flicker2, flicker3;
    boolean cycleRunning = false;
    int cycleMode = 0;
    double cycleStartTime = 0;

    @Override
    public void init() {
        // Shooter
        shooterMotor = hardwareMap.get(DcMotor.class, "testemotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flickers (optional)
        flicker1 = hardwareMap.get(Servo.class, "lift2");
        flicker2 = hardwareMap.get(Servo.class, "lift1");
        flicker3 = hardwareMap.get(Servo.class, "lift3");

        flicker1.setPosition(0.05);
        flicker2.setPosition(0.745);
        flicker3.setPosition(0.1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ----------------------------
        // Shooter control
        // ----------------------------
        boolean rtPressed = gamepad1.right_trigger > 0.5;
        boolean ltPressed = gamepad1.left_trigger > 0.5;

        // Increment/decrement power
        if (rtPressed && !rtWasPressed) shooterPower += step;
        if (ltPressed && !ltWasPressed) shooterPower -= step;

        rtWasPressed = rtPressed;
        ltWasPressed = ltPressed;

        // Presets
        if (gamepad1.a) shooterPower = 1.0; // full power
        if (gamepad1.b) shooterPower = 0.0; // stop

        // Clamp power
        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

        // Apply
        shooterMotor.setPower(shooterPower);

        // ----------------------------
        // Flickers (optional)
        // ----------------------------
        if (gamepad1.dpad_left && !cycleRunning) {
            cycleRunning = true;
            cycleMode = 1;
            cycleStartTime = getRuntime();
        }
        if (gamepad1.dpad_right && !cycleRunning) {
            cycleRunning = true;
            cycleMode = 2;
            cycleStartTime = getRuntime();
        }
        if (gamepad1.dpad_down && !cycleRunning) {
            cycleRunning = true;
            cycleMode = 3;
            cycleStartTime = getRuntime();
        }

        if (cycleRunning) {
            double t = (getRuntime() - cycleStartTime) * 1000; // ms
            if (cycleMode == 1) {
                if (t < 200) flicker1.setPosition(0.55);
                else if (t < 400) flicker1.setPosition(0.05);
                else cycleRunning = false;
            } else if (cycleMode == 2) {
                if (t < 200) flicker2.setPosition(0.30);
                else if (t < 400) flicker2.setPosition(0.745);
                else cycleRunning = false;
            } else if (cycleMode == 3) {
                if (t < 200) flicker3.setPosition(0.55);
                else if (t < 400) flicker3.setPosition(0.1);
                else cycleRunning = false;
            }
        }

        // ----------------------------
        // Telemetry
        // ----------------------------
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }
}
