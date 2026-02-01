package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.EnumMap;


@TeleOp(name = "RevColor3Sensors")
public class RevColorPurpleGreen extends LinearOpMode {
    public enum ColorType {
        GREEN, PURPLE, UNKNOWN
    }

//    private static final String SENSOR_1_NAME = "color1";
    private static final String SENSOR_2_NAME = "color2";
    private static final String SENSOR_3_NAME = "color3";

//    private NormalizedColorSensor color1;
    private NormalizedColorSensor color2;
    private NormalizedColorSensor color3;

    Servo led = hardwareMap.get(Servo.class,"indicator");


    @Override
    public void runOpMode() throws InterruptedException {

//        color1 = hardwareMap.get(NormalizedColorSensor.class, SENSOR_1_NAME);
        color2 = hardwareMap.get(NormalizedColorSensor.class, SENSOR_2_NAME);
        color3 = hardwareMap.get(NormalizedColorSensor.class, SENSOR_3_NAME);

        EnumMap<colorIndicator.Slot, colorIndicator.ColorType> colors = new EnumMap<>(colorIndicator.Slot.class);

        IndicatorBlinker blinker = new IndicatorBlinker(this,led);

        waitForStart();

        while (opModeIsActive()) {

//            detectAndTelemetry(color1, "color1");
            colorIndicator.ColorType left = detectAndTelemetry(color2, "color2");
            colorIndicator.ColorType back = detectAndTelemetry(color3, "color3");
            colorIndicator.ColorType right = colorIndicator.inferRight(left,back);
            colorIndicator.Slot[] order = colorIndicator.computeOrder(left,back,colorIndicator.OrderPolicy.PURPLES_THEN_GREENS);
            colors.put(colorIndicator.Slot.LEFT,left);
            colors.put(colorIndicator.Slot.BACK,back);
            colors.put(colorIndicator.Slot.RIGHT,right);
            blinker.blinkSequence(order,colors,500,100,5);

            blinker.stop();

            telemetry.update();
            sleep(50);
        }
    }

    private colorIndicator.ColorType detectAndTelemetry(NormalizedColorSensor sensor, String name) {

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

        if (isPurple == isGreen) {
            return colorIndicator.ColorType.UNKNOWN;   // both true OR both false
        }
        return isPurple ? colorIndicator.ColorType.PURPLE : colorIndicator.ColorType.GREEN;


    }

    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
