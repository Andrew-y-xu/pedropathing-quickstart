package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AutoIntake;
import org.firstinspires.ftc.teamcode.common.AutoFlicker;
import org.firstinspires.ftc.teamcode.common.AutoVision;
import org.firstinspires.ftc.teamcode.common.AutoAim;
import org.firstinspires.ftc.teamcode.common.AutoShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuningPath.TestPedroPath1;
import org.firstinspires.ftc.teamcode.pedroPathing.tuningPath.AutonomousPath;


//@Disabled //Comment and UnComment @Autonomous to re-enable
@Autonomous
public class RealMeet3_v3RedFarAuto extends OpMode {
    ElapsedTime timer = new ElapsedTime();

    //--- Hardware (Drive, Intake, Chamber, Limelight, Turret)
    Follower follower;
    DcMotor intake;
    Servo intake2;
    Servo slot1;
    Servo slot2;
    Servo slot3;
    Limelight3A limelight;
    DcMotor turretmotor;
    DcMotor flywheel;
    Servo hoodservo;

    //--- Component classes
    AutonomousPath autoPath;
    TestPedroPath1 paths;
    AutoIntake autoIntake;
    AutoFlicker autoFlicker;
    AutoVision autoVision;
    AutoAim autoAim;
    AutoShooter autoShoot;

    double derivativeTx = 0;
    double lastTx = 0;
    double lastTimeUpdated = 0;
    double pPID = 0.04;
    double dPID = 0.001;

    double limelight_tx = 0;
    double limelight_ty = 0;
    String limelightMessage;
    double shooterPowerValue = 0;
    double servoPositionValue = 0.91;



    /***************************/
    /**** init ***/
    /***************************/
    public void init(){
        telemetry.addLine("Initialized Red Far Auto!");
        // init Pathing
        follower = Constants.createFollower(hardwareMap);
        paths = new TestPedroPath1(follower);
        follower.setStartingPose(new Pose(123.5, 127, Math.toRadians(35.5)));
        follower.setMaxPower(0.95);
        // init Hardware
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        intake2=hardwareMap.get(Servo.class, "intake2servo");
        turretmotor = hardwareMap.get(DcMotor.class, "turretmotor");

        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(0.68);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        lastTimeUpdated = System.nanoTime();

        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        autoFlicker = new AutoFlicker(slot1, slot2, slot3);
        autoIntake  = new AutoIntake(intake, intake2);
        autoShoot   = new AutoShooter();
        autoPath    = new AutonomousPath(follower, flywheel, autoFlicker);
        telemetry.update();
    }

    /***************************/
    /**** start ***/
    /***************************/
    public void start(){
        autoShoot.setFlywheelPower(0.61);
        this.shooterPowerValue = autoShoot.getFlywheelPower();
        flywheel.setPower(this.shooterPowerValue);
        autoIntake.startIntake();
    }

    /***************************/
    /**** loop ***/
    /***************************/
    public void loop(){
        //flywheel.setPower(0.61);
        follower.update();
        autoFlicker.update();
        autoIntake.update();

        try {
            autoPath.update();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("----- Auto Path -----", null);
        telemetry.addData("CurrentPath(autoPath): ", autoPath);

        telemetry.addData("----- Shooter Data -----", null);
        telemetry.addData("AutoShoot(FlyWheel Power): ", this.shooterPowerValue);
        telemetry.addData("AutoShoot(Hood Position): ", this.servoPositionValue);

        telemetry.addData("----- Limelight Data -----", null);
        telemetry.addData("Limelight: ", limelightMessage);
        telemetry.addData("LimeLight(ty): ", limelight_ty);
        telemetry.addData("LimeLight(tx): ", limelight_tx);

        telemetry.addData("Timer (s)", timer.seconds());
        telemetry.update();


    }

}