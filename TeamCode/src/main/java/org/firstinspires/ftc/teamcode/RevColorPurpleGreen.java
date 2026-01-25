package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "RevColor3Sensors")
public class RevColorPurpleGreen extends LinearOpMode {

    private static final String SENSOR_1_NAME = "ColorL";
    private static final String SENSOR_2_NAME = "ColorR";
    private static final String SENSOR_3_NAME = "ColorB";

    private NormalizedColorSensor colorL;
    private NormalizedColorSensor colorR;
    private NormalizedColorSensor colorB;

    @Override
    public void runOpMode() {

        colorL = hardwareMap.get(NormalizedColorSensor.class, SENSOR_1_NAME);
        colorR = hardwareMap.get(NormalizedColorSensor.class, SENSOR_2_NAME);
        colorB = hardwareMap.get(NormalizedColorSensor.class, SENSOR_3_NAME);

        waitForStart();

        while (opModeIsActive()) {

            detectAndTelemetry(colorL, "colorL");
            detectAndTelemetry(colorR, "colorR");
            detectAndTelemetry(colorB, "colorB");

            telemetry.update();
            sleep(50);
        }
    }

    private void detectAndTelemetry(NormalizedColorSensor sensor, String name) {

        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double r = clamp01(rgba.red);
        double g = clamp01(rgba.green);
        double b = clamp01(rgba.blue);

        double intensity = r + g + b;
        boolean brightEnough = intensity > 0.06;
        boolean notBlownOut = intensity < 2.7;

        double sum = Math.max(1e-6, intensity);
        double rn = r / sum;
        double gn = g / sum;
        double bn = b / sum;

        boolean isGreen =
                brightEnough && notBlownOut &&
                        gn > 0.45 &&
                        gn > rn + 0.05 &&
                        gn > bn + 0.05;

        boolean isPurple =
                brightEnough && notBlownOut &&
                        rn > 0.20 &&
                        bn > 0.40 &&
                        gn < 0.35 &&
                        Math.abs(rn - bn) < 0.30;

        telemetry.addLine("Sensor " + name + ":");
        telemetry.addData("  green", isGreen);
        telemetry.addData("  purple", isPurple);
    }

    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
