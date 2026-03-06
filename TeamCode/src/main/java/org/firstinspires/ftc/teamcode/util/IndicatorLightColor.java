package org.firstinspires.ftc.teamcode.util;

/**

 Preset colors for the goBILDA RGB Indicator Light
 (3118-0808-0002). Values are FTC servo positions (0–1).*
 The light smoothly interpolates between these anchor
 points, so any value in [0.277, 0.722] produces a valid
 color. Values below 0.277 turn the light off; values
 above 0.722 transition toward white.*/
public enum IndicatorLightColor {
    OFF(0.0),
    RED(0.277),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.505),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.701),
    WHITE(1.0);

    private final double position;

    IndicatorLightColor(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}