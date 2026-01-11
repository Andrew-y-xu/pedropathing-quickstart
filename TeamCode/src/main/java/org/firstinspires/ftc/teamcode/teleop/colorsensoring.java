package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class colorsensoring {

    public static class Result {
        public boolean isGreen;
        public boolean isPurple;
        public Result(boolean g, boolean p) { isGreen = g; isPurple = p; }
    }

    public static Result classify(NormalizedRGBA rgba) {
        double r = Math.max(0.0, Math.min(1.0, rgba.red));
        double g = Math.max(0.0, Math.min(1.0, rgba.green));
        double b = Math.max(0.0, Math.min(1.0, rgba.blue));

        double intensity = r + g + b;
        boolean brightEnough = intensity > 0.06;
        boolean notBlownOut = intensity < 2.7;

        double sum = Math.max(1e-6, r + g + b);
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

        return new Result(isGreen, isPurple);
    }
}
