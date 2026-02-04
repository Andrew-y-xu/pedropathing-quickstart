package org.firstinspires.ftc.teamcode.pedroPathing.tuningPath;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.AutoFlicker;


public class AutonomousPath {
    private int pathState;
    DcMotor flywheel;
    Follower follower;
    TestPedroPath1 paths;
    AutoFlicker autoFlicker;

    public AutonomousPath(Follower follower, DcMotor flywheel, AutoFlicker autoFlicker) {
        this.pathState = 0;
        this.flywheel = flywheel;
        this.follower = follower;
        this.autoFlicker = autoFlicker;
    }

    public int getPathState() {
        return this.pathState;
    }

    public void update() throws InterruptedException {
        //autoShoot.advancedMathematics(limelight_ty);
        //this.shooterPowerValue = autoShoot.getFlywheelPower();
        //this.servoPositionValue = autoShoot.getAnglePosition();
        //flywheel.setPower(shooterPowerValue);
        //hoodservo.setPosition(this.servoPositionValue);

        switch (this.pathState) {
          // ---- Go to hub (ends at shared point), then flick, then leave ----
            case 0:
                this.flywheel.setPower(0.7);
                this.follower.followPath(paths.Path1);
                this.pathState = 1;
                break;

            case 1: // wait for Path1 to finish (now standing still at hub)
                if (!follower.isBusy()) {
                    this.autoFlicker.start();
                    this.pathState = 101;
                }
                break;

            case 101: // wait for flicker to finish, then go to Path2
                if (this.autoFlicker.done()) {
                    this.flywheel.setPower(0.61);
                    //intakeMode = RealMeet3_v3RedNearAuto.IntakeMode.INTAKE;
                    this.follower.setMaxPower(0.75);
                    this.follower.followPath(paths.Path2, true);
                    this.pathState = 2;
                }
                break;

            // ---- Normal travel steps (no flicker here) ----
            case 2:
                if (!this.follower.isBusy()) {
                    this.follower.setMaxPower(1.00);
                    this.follower.followPath(paths.Path3, true);
                    this.pathState = 4;
                }
                break;

      //            case 3:
      //                if (!follower.isBusy()) {
      //                    startIntakePulse();
      //                    follower.followPath(paths.Path4, true); // ends at hub
      //                    pathState = 4;
      //                }
      //                break;

            // ---- Arrived at hub again -> flick -> then Path5 ----
            case 4:
                if (!this.follower.isBusy()) { // standing still at hub
                    this.autoFlicker.start();
                    this.pathState = 401;
                }
                break;

            case 401:
                if (this.autoFlicker.done()) {
                    //intakeMode = RealMeet3_v3RedNearAuto.IntakeMode.INTAKE;
                    this.follower.setMaxPower(0.75);
                    this.follower.followPath(paths.Path4, true);
                    this.pathState = 5;
                }
                break;

            case 5:
                if (!this.follower.isBusy()) {
                    //startIntakePulse();
                    this.follower.setMaxPower(1.00);
                    this.follower.followPath(paths.Path5, true); // ends at hub
                    this.pathState = 6;
                }
                break;

            // ---- Arrived at hub -> flick -> then Path7 ----
            case 6:
                if (!this.follower.isBusy()) { // standing still at hub
                    this.autoFlicker.start();
                    this.pathState = 601;
                }
                break;

            case 601:
                if (this.autoFlicker.done()) {
                    //intakeMode = RealMeet3_v3RedNearAuto.IntakeMode.INTAKE;
                    this.follower.setMaxPower(0.75);
                    this.follower.followPath(paths.Path6, true);
                    this.pathState = 7;
                }
                break;

            case 7:
                if (!this.follower.isBusy()) {
                    //startIntakePulse();
                    this.follower.setMaxPower(1.00);
                    this.follower.followPath(paths.Path7, true); // ends at hub
                    this.pathState = 8;
                }
                break;

            // ---- Arrived at hub -> flick -> then Path9 ----
            case 8:
                if (!this.follower.isBusy()) { // standing still at hub
                    this.autoFlicker.start();
                    this.pathState = 801;
                }
                break;

            case 801:
                if (this.autoFlicker.done()) {
                    this.follower.followPath(paths.Path8, true);
                    this.pathState = 9;
                }
                break;

            case 9:
                if (!this.follower.isBusy()) {
                    this.pathState = -1;
                }
                break;
        }
    }


}
