package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Wrapper around a goBILDA RGB Indicator Light (3118-0808-0002)
 * connected to a REV Hub servo port.
 *
 * Usage:
 *   IndicatorLight light = new IndicatorLight(hardwareMap, "light1");
 *   light.setColor(IndicatorLightColor.GREEN);
 *   light.off();
 */
public class IndicatorLight {

    private final Servo servo;
    private IndicatorLightColor currentColor = IndicatorLightColor.OFF;

    // ── Constructors ────────────────────────────────────

    /** Create from a hardware map name configured in the Driver Station. */
    public IndicatorLight(HardwareMap hwMap, String deviceName) {
        this.servo = hwMap.get(Servo.class, deviceName);
        off(); // start in a known state
    }

    /** Create from an already-obtained Servo reference. */
    public IndicatorLight(Servo servo) {
        this.servo = servo;
        off();
    }

    // ── Core API ────────────────────────────────────────

    /** Set the light to a predefined color. */
    public void setColor(IndicatorLightColor color) {
        this.currentColor = color;
        servo.setPosition(color.getPosition());
    }

    /**
     * Set the light to an arbitrary FTC position (0.0–1.0).
     * Useful for colors between the predefined anchor points.
     * Values below 0.277 turn the light off.
     * Values above 0.722 blend toward white.
     */
    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        this.currentColor = null; // custom position
        servo.setPosition(position);
    }

    /** Turn the light off. */
    public void off() {
        setColor(IndicatorLightColor.OFF);
    }

    /** Get the last color that was set (null if a custom position was used). */
    public IndicatorLightColor getCurrentColor() {
        return currentColor;
    }

    /** Get the underlying servo position. */
    public double getPosition() {
        return servo.getPosition();
    }

    // ── Convenience shortcuts ───────────────────────────

    public void red()    { setColor(IndicatorLightColor.RED); }
    public void orange() { setColor(IndicatorLightColor.ORANGE); }
    public void yellow() { setColor(IndicatorLightColor.YELLOW); }
    public void sage()   { setColor(IndicatorLightColor.SAGE); }
    public void green()  { setColor(IndicatorLightColor.GREEN); }
    public void azure()  { setColor(IndicatorLightColor.AZURE); }
    public void blue()   { setColor(IndicatorLightColor.BLUE); }
    public void indigo() { setColor(IndicatorLightColor.INDIGO); }
    public void violet() { setColor(IndicatorLightColor.VIOLET); }
    public void white()  { setColor(IndicatorLightColor.WHITE); }

    // ── Conditional helpers (the real power) ────────────

    /**
     * Set color based on a boolean condition.
     * Example: light.setIf(isAligned, GREEN, RED);
     */
    public void setIf(
            boolean condition,
            IndicatorLightColor ifTrue,
            IndicatorLightColor ifFalse
    ) {
        setColor(condition ? ifTrue : ifFalse);
    }

    /**
     * Choose a color from a set of thresholds.
     * Thresholds are evaluated in order; the first match wins.
     *
     * Example — distance sensor feedback:
     *   light.setFromThresholds(distance,
     *       new Threshold(10, GREEN),   // ≤ 10 cm
     *       new Threshold(30, YELLOW),  // ≤ 30 cm
     *       new Threshold(60, ORANGE)   // ≤ 60 cm
     *   );
     *   // anything > 60 → fallback RED
     */
    public void setFromThresholds(
            double value,
            IndicatorLightColor fallback,
            Threshold... thresholds
    ) {
        for (Threshold t : thresholds) {
            if (value <= t.upperBound) {
                setColor(t.color);
                return;
            }
        }
        setColor(fallback);
    }

    // ── Threshold helper record ─────────────────────────

    public static class Threshold {
        public final double upperBound;
        public final IndicatorLightColor color;

        public Threshold(
                double upperBound,
                IndicatorLightColor color
        ) {
            this.upperBound = upperBound;
            this.color = color;
        }
    }
}