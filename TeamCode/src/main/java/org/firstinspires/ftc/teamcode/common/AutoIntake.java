package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoIntake {
    enum IntakeMode {
        INTAKE,
        OUTTAKE,
        OFF
    }
    IntakeMode intakeMode = IntakeMode.INTAKE;
    // Intake pulse state
    private boolean intakePulseActive = false;
    private long intakePulseStart = 0;
    private boolean intakePulseEnabled = false;
    // tune this
    private long intakePulseTime = 600; // ms
    DcMotor intake1;
    Servo intake2;


    public AutoIntake(DcMotor intake1, Servo intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
    }


    public void startIntake() {
        if (!intakePulseEnabled) return;
        if (intakePulseActive) return;

        intakePulseActive = true;
        intakePulseStart = System.currentTimeMillis();
        //--- original setting which looks more like OUTTAKE
        //this.intake1.setPower(1);
        //this.intake2.setPosition(0);
        //--- Updated settings for INTAKE
        this.intakeMode = IntakeMode.INTAKE;
        update();
    }

    public void startOutake() {
        if (!intakePulseEnabled) return;
        if (intakePulseActive) return;
        intakePulseActive = true;
        intakePulseStart = System.currentTimeMillis();
        this.intakeMode = IntakeMode.OUTTAKE;
        update();
    }

    public void stop() {
        if (!intakePulseEnabled) return;
        if (!intakePulseActive) return; //If not running, no need to stop
        intakePulseActive = false;
        intakePulseStart = 0;
        this.intakeMode = IntakeMode.OFF;
        update();
    }

    public void update() {
        if (intakePulseActive) {
            // Intake pulse is controlling intake right now.
            // Do nothing here so it doesn't get overridden.
        } else {
            switch (intakeMode) {
                case INTAKE:
                    intake1.setPower(-1);
                    intake2.setPosition(1);
                    break;

                case OUTTAKE:
                    intake1.setPower(1);
                    intake2.setPosition(0);
                    break;

                case OFF:
                    intake1.setPower(0);
                    intake2.setPosition(0.5);
                    break;
            }
        }
    }

    //--- original logic
    public void updateIntakePulse() {
        if (!intakePulseActive) return;

        if (System.currentTimeMillis() - intakePulseStart >= intakePulseTime) {
            this.intake1.setPower(-1);     // return to normal intake
            this.intake2.setPosition(1);
            intakePulseActive = false;
        }
    }

}
