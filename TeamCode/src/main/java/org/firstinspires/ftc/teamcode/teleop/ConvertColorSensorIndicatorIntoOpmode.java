package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.util.IndicatorLight;

@TeleOp(name = "ColorSensorIndicatorOpmode")
public class ConvertColorSensorIndicatorIntoOpmode extends OpMode {

    private static final String SENSOR1_NAME = "ColorSensor1";
    private static final String SENSOR2_NAME = "ColorSensor2";

    private static final String INDICATOR1_NAME = "IndicatorServo1";
    private static final String INDICATOR2_NAME = "IndicatorServo2";

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;

    private IndicatorLight light1;
    private IndicatorLight light2;

    // Store last locked colors independently
    private String lastLockedColor1 = "unknown";
    private String lastLockedColor2 = "unknown";

    @Override
    public void init() {
        // --- Hardware setup ---
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, SENSOR1_NAME);
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, SENSOR2_NAME);

        light1 = new IndicatorLight(hardwareMap, INDICATOR1_NAME);
        light2 = new IndicatorLight(hardwareMap, INDICATOR2_NAME);
    }

    @Override
    public void loop() {
        String detected1 = detectColor(colorSensor1);
        String detected2 = detectColor(colorSensor2);

        // Locking code (Sensor 1)
        if (detected1.equals("green") || detected1.equals("purple")) {
            lastLockedColor1 = detected1;
        }

        // Locking code (Sensor 2)
        if (detected2.equals("green") || detected2.equals("purple")) {
            lastLockedColor2 = detected2;
        }

        // Display locked colors on LED
        // Sensor 1 LED
        if (lastLockedColor1.equals("green")) {
            light1.green();
        } else if (lastLockedColor1.equals("purple")) {
            light1.violet();
        }

        // Sensor 2 LED
        if (lastLockedColor2.equals("green")) {
            light2.green();
        } else if (lastLockedColor2.equals("purple")) {
            light2.violet();
        }

        telemetry.addData("Sensor1 Detected", detected1);
        telemetry.addData("Sensor1 Locked", lastLockedColor1);
        telemetry.addData("Sensor2 Detected", detected2);
        telemetry.addData("Sensor2 Locked", lastLockedColor2);
        telemetry.update();
    }

    private String detectColor(NormalizedColorSensor sensor) {
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

        boolean isGreen = brightEnough && notBlownOut &&
                gn > 0.45 &&
                gn > rn + 0.05 &&
                gn > bn + 0.05;

        boolean isPurple = brightEnough && notBlownOut &&
                rn > 0.20 &&
                bn > 0.40 &&
                gn < 0.35 &&
                Math.abs(rn - bn) < 0.30;

        if (isGreen) return "green";
        if (isPurple) return "purple";
        return "unknown"; // does NOT reset lock
    }

    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}