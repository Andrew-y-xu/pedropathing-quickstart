package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Brushland 3 Sensor Test")
public class ColorSensorNServo extends OpMode {

    DigitalChannel sA_pin0, sA_pin1;
    DigitalChannel sB_pin0, sB_pin1;
    DigitalChannel sC_pin0, sC_pin1;

    @Override
    public void init() {


        // === Sensor A ===
        sA_pin0 = hardwareMap.get(DigitalChannel.class, "digital_0A");
        sA_pin1 = hardwareMap.get(DigitalChannel.class, "digital_1A");
        sA_pin0.setMode(DigitalChannel.Mode.INPUT);
        sA_pin1.setMode(DigitalChannel.Mode.INPUT);

        // === Sensor B ===
        sB_pin0 = hardwareMap.get(DigitalChannel.class, "digital_0B");
        sB_pin1 = hardwareMap.get(DigitalChannel.class, "digital_1B");
        sB_pin0.setMode(DigitalChannel.Mode.INPUT);
        sB_pin1.setMode(DigitalChannel.Mode.INPUT);

        // === Sensor C ===
        sC_pin0 = hardwareMap.get(DigitalChannel.class, "digital_0C");
        sC_pin1 = hardwareMap.get(DigitalChannel.class, "digital_1C");
        sC_pin0.setMode(DigitalChannel.Mode.INPUT);
        sC_pin1.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized (3 Sensors)");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addLine("=== Sensor A ===");
        telemetry.addData("A Pin0", sA_pin0.getState());
        telemetry.addData("A Pin1", sA_pin1.getState());

        telemetry.addLine("=== Sensor B ===");
        telemetry.addData("B Pin0", sB_pin0.getState());
        telemetry.addData("B Pin1", sB_pin1.getState());

        telemetry.addLine("=== Sensor C ===");
        telemetry.addData("C Pin0", sC_pin0.getState());
        telemetry.addData("C Pin1", sC_pin1.getState());

        telemetry.update();
    }
}
