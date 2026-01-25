package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.colorIndicator;

import java.util.Map;

public class IndicatorBlinker {

    // Tune these to your indicator’s PWM mapping
    private static final double POS_OFF = 0.05;
    private static final double POS_GREEN = 0.40;
    private static final double POS_PURPLE = 0.80;

    private final LinearOpMode op;
    private final Servo indicator;

    public IndicatorBlinker(LinearOpMode op, Servo indicator) {
        this.op = op;
        this.indicator = indicator;
        setOff();
    }

    // Solid color helpers
    public void setSolid(colorIndicator.ColorType color) {
        switch (color) {
            case GREEN:
                indicator.setPosition(POS_GREEN);
                break;
            case PURPLE:
                indicator.setPosition(POS_PURPLE);
                break;
            default:
                indicator.setPosition(POS_OFF);
                break;
        }
    }

    public void setOff() {
        indicator.setPosition(POS_OFF);
    }

    // Blink a single color: onMs each cycle, offMs gap, repeated 'repeats' times.
    public void blinkColor(
            colorIndicator.ColorType color, int onMs, int offMs, int repeats)
            throws InterruptedException {
        for (int i = 0; i < repeats && op.opModeIsActive(); i++) {
            setSolid(color);
            sleepCoop(onMs);
            setOff();
            if (offMs > 0) sleepCoop(offMs);
        }
    }

    // Blink a sequence that matches your Slot order and slot->color map.
    // Example: order = [LEFT, BACK, RIGHT], colors = {LEFT: GREEN, BACK: PURPLE, RIGHT: PURPLE}
    // Each item lights for stepMs, then a gap of gapMs. Repeat 'repeats' times (use repeats=1 to show once).
    public void blinkSequence(
            colorIndicator.Slot[] order,
            Map<colorIndicator.Slot, colorIndicator.ColorType> slotColors,
            int stepMs,
            int gapMs,
            int repeats)
            throws InterruptedException {

        for (int r = 0; r < repeats && op.opModeIsActive(); r++) {
            for (colorIndicator.Slot s : order) {
                if (!op.opModeIsActive()) break;

                colorIndicator.ColorType color =
                        slotColors.getOrDefault(s, colorIndicator.ColorType.UNKNOWN);

                setSolid(color);
                sleepCoop(stepMs);
                setOff();
                if (gapMs > 0) sleepCoop(gapMs);
            }
            // optional extra gap between full cycles:
            // sleepCoop(200);
        }
    }

    // Convenience: blink continuously until a stop condition becomes true
    public void blinkSequenceUntil(
            colorIndicator.Slot[] order,
            Map<colorIndicator.Slot, colorIndicator.ColorType> slotColors,
            int stepMs,
            int gapMs,
            java.util.function.BooleanSupplier stopCondition)
            throws InterruptedException {

        while (op.opModeIsActive() && !stopCondition.getAsBoolean()) {
            for (colorIndicator.Slot s : order) {
                if (!op.opModeIsActive() || stopCondition.getAsBoolean()) break;

                colorIndicator.ColorType color =
                        slotColors.getOrDefault(s, colorIndicator.ColorType.UNKNOWN);

                setSolid(color);
                sleepCoop(stepMs);
                setOff();
                if (gapMs > 0) sleepCoop(gapMs);
            }
        }
        setOff();
    }

    public void stop() {
        setOff();
    }

    private void sleepCoop(int ms) throws InterruptedException {
        long end = System.nanoTime() + ms * 1_000_000L;
        while (op.opModeIsActive() && System.nanoTime() < end) {
            op.idle();
        }
    }
}