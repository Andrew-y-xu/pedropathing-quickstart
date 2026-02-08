package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "RevColorSensor")
@Disabled
public class RevColorSensor extends LinearOpMode {

    // Change to match the name you configured in the Driver Station
    private static final String SENSOR_NAME = "Color1R";

    private NormalizedColorSensor color;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(NormalizedColorSensor.class, SENSOR_NAME);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA rgba = color.getNormalizedColors();

            // Raw normalized values are 0..1; clamp and convert to 0..1 doubles
            double r = clamp01(rgba.red);
            double g = clamp01(rgba.green);
            double b = clamp01(rgba.blue);

            // Intensity guard (avoid classifying very dark/very bright washout)
            double intensity = r + g + b; // 0..3
            boolean brightEnough = intensity > 0.06; // tune: ~0.06 = avg 0.02 per channel
            boolean notBlownOut = intensity < 2.7;   // optional upper guard

            // Normalize to sum so ratios are lighting-invariant
            double sum = Math.max(1e-6, r + g + b);
            double rn = r / sum;
            double gn = g / sum;
            double bn = b / sum;

            // Simple RGB-ratio :
            // - Green: G dominates, and R/B are clearly lower
            // - Purple: R and B are both relatively high while G is clearly lower
            boolean isGreen =
                    brightEnough && notBlownOut &&
                            gn > 0.45 &&            // green is at least ~45% of total
                            gn > rn + 0.05 &&       // green exceeds red by margin
                            gn > bn + 0.05;         // green exceeds blue by margin

            boolean isPurple =
                    brightEnough && notBlownOut &&
                            rn > 0.20 && bn > 0.40 && // both red and blue are reasonably strong
                            gn < 0.35 &&              // green is relatively suppressed
                            Math.abs(rn - bn) < 0.30; // red and blue not too far apart

            telemetry.addData("purple", isPurple);
            telemetry.addData("green", isGreen);

            // Debug info to help tuning
            /*telemetry.addData("R", "%.3f", r);
            telemetry.addData("G", "%.3f", g);
            telemetry.addData("B", "%.3f", b);
            telemetry.addData("Sum", "%.3f", sum);
            telemetry.addData("rn/gn/bn", "%.2f / %.2f / %.2f", rn, gn, bn);*/
            telemetry.update();

            sleep(50);
        }
    }

    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}