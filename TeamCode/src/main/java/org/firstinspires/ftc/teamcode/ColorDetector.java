package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.DetectableColor;
import org.firstinspires.ftc.teamcode.util.ColorRangefinder;

public class ColorDetector {

    public ColorRangefinder crf;
    public DigitalChannel digitalPin0;
    public DigitalChannel digitalPin1;
    public AnalogInput analogPin0;
    public DetectableColor pin0Color;
    public DetectableColor pin1Color;


    public ColorDetector() {}

    public static ColorDetector forConfiguration(
            HardwareMap hardwareMap,
            String deviceName
    ) {
        ColorDetector cd = new ColorDetector();
        cd.crf = new ColorRangefinder(
                hardwareMap.get(RevColorSensorV3.class, deviceName)
        );
        return cd;
    }

    public static ColorDetector forDigitalRead(
            HardwareMap hardwareMap,
            String pin0Name,
            String pin1Name,
            DetectableColor pin0Color,
            DetectableColor pin1Color
    ) {
        ColorDetector cd = new ColorDetector();
        cd.digitalPin0 =
                hardwareMap.digitalChannel.get(pin0Name);
        cd.digitalPin1 =
                hardwareMap.digitalChannel.get(pin1Name);
        cd.pin0Color = pin0Color;
        cd.pin1Color = pin1Color;
        return cd;
    }

    public static ColorDetector forAnalogRead(
            HardwareMap hardwareMap,
            String analogName
    ) {
        ColorDetector cd = new ColorDetector();
        cd.analogPin0 =
                hardwareMap.analogInput.get(analogName);
        return cd;
    }


    /**
     * Write a two-color digital configuration to the sensor.
     * Both pins must be configured at the same time.
     *
     * @param pin0Color  color to detect on pin 0
     * @param pin1Color  color to detect on pin 1
     * @param maxDistMm  maximum detection distance in mm
     *                   (set to 0 to disable distance gating)
     */
    public void configureTwoColorDigital(
            DetectableColor pin0Color,
            DetectableColor pin1Color,
            double maxDistMm
    ) {
        requireI2c();

        crf.setPin0Digital(
                ColorRangefinder.DigitalMode.HSV,
                pin0Color.getLowerBound(),
                pin0Color.getUpperBound()
        );
        if (pin0Color.requiresInvertHue()) {
            crf.setPin0InvertHue();
        }

        crf.setPin1Digital(
                ColorRangefinder.DigitalMode.HSV,
                pin1Color.getLowerBound(),
                pin1Color.getUpperBound()
        );
        if (pin1Color.requiresInvertHue()) {
            crf.setPin1InvertHue();
        }

        if (maxDistMm > 0) {
            crf.setPin0DigitalMaxDistance(
                    ColorRangefinder.DigitalMode.HSV, maxDistMm
            );
            crf.setPin1DigitalMaxDistance(
                    ColorRangefinder.DigitalMode.HSV, maxDistMm
            );
        }
    }

    /**
     * Write an analog HSV + digital placeholder configuration.
     * Pin 0 outputs analog HSV hue; pin 1 is a digital
     * placeholder (required so the sensor doesn't revert to
     * I2C).
     */
    public void configureAnalogHsv() {
        requireI2c();
        crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        crf.setPin1Digital(
                ColorRangefinder.DigitalMode.HSV, 0, 20
        );
    }

    /**
     * Write an analog distance configuration.
     * Pin 0 outputs analog distance; pin 1 is a digital
     * placeholder.
     */
    public void configureAnalogDistance() {
        requireI2c();
        crf.setPin0Analog(
                ColorRangefinder.AnalogMode.DISTANCE
        );
        crf.setPin1Digital(
                ColorRangefinder.DigitalMode.DISTANCE, 0, 20
        );
    }

    public void configureColorAndDistance(
            DetectableColor color,
            double maxDistMm,
            double distThresh
    ) {
        requireI2c();

        crf.setPin0Digital(
                ColorRangefinder.DigitalMode.HSV,
                color.getLowerBound(),
                color.getUpperBound()
        );
        if (color.requiresInvertHue()) {
            crf.setPin0InvertHue();
        }
        if (maxDistMm > 0) {
            crf.setPin0DigitalMaxDistance(
                    ColorRangefinder.DigitalMode.HSV, maxDistMm
            );
        }

        crf.setPin1Digital(
                ColorRangefinder.DigitalMode.DISTANCE,
                0,
                distThresh
        );
    }


    public void addPin0Color(DetectableColor color) {
        requireI2c();
        crf.setPin0Digital(
                ColorRangefinder.DigitalMode.HSV,
                color.getLowerBound(),
                color.getUpperBound()
        );
    }

    /**
     * Add an extra color threshold to pin 1.
     */
    public void addPin1Color(DetectableColor color) {
        requireI2c();
        crf.setPin1Digital(
                ColorRangefinder.DigitalMode.HSV,
                color.getLowerBound(),
                color.getUpperBound()
        );
    }

    /**
     * Set the LED brightness (0-100). Nonlinear; values below
     * 30 are noticeably dimmer.
     */
    public void setLedBrightness(int brightness) {
        requireI2c();
        crf.setLedBrightness(brightness);
    }

    // ────────────────────────────────────────────
    //  Digital reads (match time)
    // ────────────────────────────────────────────

    /** @return true if pin 0 threshold is triggered. */
    public boolean isPin0Active() {
        requireDigital();
        return digitalPin0.getState();
    }

    /** @return true if pin 1 threshold is triggered. */
    public boolean isPin1Active() {
        requireDigital();
        return digitalPin1.getState();
    }

    /**
     * Identify which configured color is currently detected.
     *
     * @return the matching DetectableColor, or null if neither
     *         pin is active
     */
    public DetectableColor getDetectedColor() {
        requireDigital();
        boolean p0 = digitalPin0.getState();
        boolean p1 = digitalPin1.getState();

        if (p0 && p1) {
            return pin0Color;
        } else if (p0) {
            return pin0Color;
        } else if (p1) {
            return pin1Color;
        }
        return null;
    }

    /**
     * Check if a specific color is currently detected on
     * either pin.
     */
    public boolean isColorDetected(DetectableColor color) {
        requireDigital();
        if (color == pin0Color && digitalPin0.getState()) {
            return true;
        }
        if (color == pin1Color && digitalPin1.getState()) {
            return true;
        }
        return false;
    }

    // ────────────────────────────────────────────
    //  Analog reads (match time)
    // ────────────────────────────────────────────

    /** @return raw voltage from analog pin 0 (0-3.3V). */
    public double getAnalogVoltage() {
        requireAnalog();
        return analogPin0.getVoltage();
    }

    /** @return HSV hue in degrees (0-360). */
    public double getAnalogHue() {
        return getAnalogVoltage() / 3.3 * 360.0;
    }

    /** @return distance in millimeters (0-100). */
    public double getAnalogDistanceMm() {
        return getAnalogVoltage() / 3.3 * 100.0;
    }

    // ────────────────────────────────────────────
    //  Guards
    // ────────────────────────────────────────────

    private void requireI2c() {
        if (crf == null) {
            throw new IllegalStateException(
                    "ColorDetector was not created with "
                            + "forConfiguration(). Cannot write config."
            );
        }
    }

    private void requireDigital() {
        if (digitalPin0 == null || digitalPin1 == null) {
            throw new IllegalStateException(
                    "ColorDetector was not created with "
                            + "forDigitalRead(). Cannot read digital "
                            + "pins."
            );
        }
    }

    private void requireAnalog() {
        if (analogPin0 == null) {
            throw new IllegalStateException(
                    "ColorDetector was not created with "
                            + "forAnalogRead(). Cannot read analog pin."
            );
        }
    }
}
