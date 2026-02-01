package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Color Sensor Object Detect", group="Sensors")
public class RawValueDetectorREV extends LinearOpMode {

    ColorSensor colorSensor;

    final int PROX_THRESHOLD = 200;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "color1");

        telemetry.addLine("Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();   // Clear channel (overall light)
            int proximity = colorSensor.alpha();
            // On V3 alpha channel is often used as proximity/light indicator

            boolean objectDetected = proximity > PROX_THRESHOLD;

            telemetry.addLine("=== RAW SENSOR VALUES ===");
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Light", alpha);



            if (objectDetected) {
                telemetry.addLine("OBJECT DETECTED");
            } else {
                telemetry.addLine("No Object");
            }

            telemetry.update();
        }
    }
}
