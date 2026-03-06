package org.firstinspires.ftc.teamcode.util;



//Predefined color profiles for the Brushland Color Rangefinder.
//Hue bounds are in degrees (0-360) mapped to the HSV color wheel.*
//To add a custom color, add a new enum entry with
//        (lowerHueDeg, upperHueDeg, invertHue).*/
public enum DetectableColor {
    RED(0, 50, false),
    RED_INVERTED(140, 210, true),
    ORANGE(10, 40, false),
    YELLOW(55, 90, false),
    GREEN(110, 140, false),
    CYAN(160, 200, false),
    BLUE(180, 250, false),
    PURPLE(160, 190, false);

    private final double lowerBound;
    private final double upperBound;
    private final boolean invertHue;

    DetectableColor(
            double lowerHueDeg,
            double upperHueDeg,
            boolean invertHue
    ) {
        this.lowerBound = lowerHueDeg / 360.0 * 255.0;
        this.upperBound = upperHueDeg / 360.0 * 255.0;
        this.invertHue = invertHue;
    }

//    / Lower bound scaled to 0-255 for the sensor API. /
    public double getLowerBound() {
        return lowerBound;
    }

//    Upper bound scaled to 0-255 for the sensor API./
    public double getUpperBound() {
        return upperBound;
    }


//     Whether hue inversion is needed
//     (for colors near the 0/360 wrap).*/
    public boolean requiresInvertHue() {
        return invertHue;}
}