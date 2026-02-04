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

    double rightFlickerUpPosition = 0.20;
    double leftFlickerUpPosition = 0.65;
    double backFlickerUpPosition = 0.65;
    double rightFlickerDownPosition = 0.745; //Flicker servo is in reverse
    double leftFlickerDownPosition = 0.00;
    double backFlickerDownPosition = 0.1;
    // Flickers Up Position


    public AutoFlicker(Servo slot1, Servo slot2, Servo slot3) {
        flicker1 = slot1;
        flicker2 = slot2;
        flicker3 = slot3;
        flicker1.setPosition(rightFlickerDownPosition);
        flicker2.setPosition(leftFlickerDownPosition);
        flicker3.setPosition(backFlickerDownPosition);
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

            case 0: // Slot 1 up
                flicker3.setPosition(backFlickerUpPosition);
                //flicker3.setPosition(0.65);
                flickerTimer = now;
                flickerState++;
                break;

            case 1: // Slot 1 down
                if (now - flickerTimer >= upTime) {
                    flicker3.setPosition(backFlickerDownPosition);
                    //flicker3.setPosition(0.1);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 2: // Slot 2 up
                if (now - flickerTimer >= downTime) {
                    flicker2.setPosition(leftFlickerUpPosition);
                    //flicker2.setPosition(0.65);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 3: // Slot 2 down
                if (now - flickerTimer >= upTime) {
                    flicker2.setPosition(leftFlickerDownPosition);
                    //flicker2.setPosition(0);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 4: // Slot 3 up
                if (now - flickerTimer >= downTime) {
                    flicker1.setPosition(rightFlickerUpPosition);
                    //flicker1.setPosition(0.2);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 5: // Slot 3 down
                if (now - flickerTimer >= upTime) {
                    flicker1.setPosition(rightFlickerDownPosition);
                    //flicker1.setPosition(0.745);
                    flickerActive = false; // DONE
                }
                break;
        }
    }

}