package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.Servo;

public class AutoFlicker {
    private int flickerState = 0;
    private boolean flickerActive = false;
    private long flickerTimer = 0;
    long upTime = 900;
    long downTime = 800;
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    // Flickers Down Position
    double flicker1DownPosition = 0.745; //Flicker1 is in reverse
    double flicker2DownPosition = 0.00;
    double flicker3DownPosition = 0.1;
    // Flickers Up Position
    double flicker1UpPosition = 0.20;
    double flicker2UpPosition = 0.65;
    double flicker3UpPosition = 0.65;

    public AutoFlicker(Servo slot1, Servo slot2, Servo slot3) {
        flicker1 = slot1;
        flicker2 = slot2;
        flicker3 = slot3;
        flicker1.setPosition(flicker1DownPosition);
        flicker2.setPosition(flicker2DownPosition);
        flicker3.setPosition(flicker3DownPosition);
    }

    public void start() {
        if (flickerActive) return;
        flickerActive = true;
        flickerState = 0;
        flickerTimer = System.currentTimeMillis();
    }

    public boolean done() {
        return !flickerActive;
    }

    public void update() {
        if (!flickerActive) return;

        long now = System.currentTimeMillis();

        switch (flickerState) {

            //slot1 is in reverse Small mean up
            case 0: // Slot 1 up
                //flicker1.setPosition(0.10);
                flicker1.setPosition(flicker1UpPosition); //Original setting
                //flicker1.setPosition(0.30);
                flickerTimer = now;
                flickerState++;
                break;

            case 1: // Slot 1 down
                if (now - flickerTimer >= upTime) {
                    //flicker1.setPosition(0.55); //resting position too high
                    //flicker1.setPosition(0.60);
                    //flicker1.setPosition(0.65);
                    flicker1.setPosition(0.70);
                    //flicker1.setPosition(0.745); //Original setting
                    //flicker1.setPosition(flicker1DownPosition); //Original Setting with variable
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 2: // Slot 2 up
                if (now - flickerTimer >= downTime) {
                    //flicker2.setPosition(0.65); //Original setting
                    flicker2.setPosition(flicker2UpPosition);
                    //flicker2.setPosition(0.75); //Flipper too high, hitting flywheel
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 3: // Slot 2 down
                if (now - flickerTimer >= upTime) {
                    flicker2.setPosition(flicker2DownPosition); //Original setting
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 4: // Slot 3 up
                if (now - flickerTimer >= downTime) {
                    //this.flicker3.setPosition(0.65); //Original setting
                    flicker3.setPosition(flicker3UpPosition);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 5: // Slot 3 down
                if (now - flickerTimer >= upTime) {
                    flicker3.setPosition(flicker3DownPosition); //Original setting
                    flickerActive = false; // DONE
                }
                break;
        }
    }

}
