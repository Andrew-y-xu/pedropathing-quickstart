package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class colorIndicator {

    public enum Slot { LEFT, BACK, RIGHT }
    public enum ColorType { GREEN, PURPLE, UNKNOWN }

    public enum OrderPolicy {
        PHYSICAL_LBR,            // Left -> Back -> Right
        GREENS_THEN_PURPLES,     // All Greens (L->B->R), then Purples, then Unknowns
        PURPLES_THEN_GREENS      // All Purples (L->B->R), then Greens, then Unknowns
    }

    // You pass in the already-detected colors for LEFT and BACK
    public static Slot[] computeOrder(ColorType left, ColorType back, OrderPolicy orderPolicy) {
        ColorType right = inferRight(left, back);
        return buildOrder(orderPolicy, left, back, right);
    }

    // Expose the inference if you want to log/telemetry it
    public static ColorType inferRight(ColorType left, ColorType back) {
        // Rule: if back or left is GREEN, assume right is PURPLE
        if (left == ColorType.GREEN || back == ColorType.GREEN) {
            return ColorType.PURPLE;
        }
        // Fallback when neither is GREEN (e.g., both PURPLE/UNKNOWN)
        return ColorType.PURPLE; // change to GREEN/UNKNOWN if desired
    }

    private static Slot[] buildOrder(OrderPolicy policy, ColorType l, ColorType b, ColorType r) {
        switch (policy) {
            case PHYSICAL_LBR:
                return new Slot[] { Slot.LEFT, Slot.BACK, Slot.RIGHT };
            case GREENS_THEN_PURPLES:
                return concat(
                        inOrderByColor(l, b, r, ColorType.GREEN),
                        inOrderByColor(l, b, r, ColorType.PURPLE),
                        inOrderByColor(l, b, r, ColorType.UNKNOWN));
            case PURPLES_THEN_GREENS:
            default:
                return concat(
                        inOrderByColor(l, b, r, ColorType.PURPLE),
                        inOrderByColor(l, b, r, ColorType.GREEN),
                        inOrderByColor(l, b, r, ColorType.UNKNOWN));
        }
    }

    private static Slot[] inOrderByColor(ColorType l, ColorType b, ColorType r, ColorType target) {
        List<Slot> list = new ArrayList<>();
        if (l == target) list.add(Slot.LEFT);
        if (b == target) list.add(Slot.BACK);
        if (r == target) list.add(Slot.RIGHT);
        return list.toArray(new Slot[0]);
    }

    private static Slot[] concat(Slot[]... arrs) {
        List<Slot> out = new ArrayList<>();
        for (Slot[] a : arrs) for (Slot s : a) out.add(s);
        return out.toArray(new Slot[0]);
    }
}
