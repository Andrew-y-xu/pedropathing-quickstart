package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
@Disabled //Comment and UnComment @Autonomous to re-enable
//@Autonomous
public class Meet3RedFarAuto extends OpMode {
    DcMotor flywheel;
    Servo hoodservo;
    Servo slot1;
    Servo slot2;
    Servo slot3;
    DcMotor intake;
    private Follower follower;
    double derivativeTx = 0;
    private int pathState=0;
    private Paths paths;
    private Limelight3A limelight;
    DcMotor turretmotor;
    double lastTx = 0;
    double lastTimeUpdated = 0;
    double pPID = 0.04;
    double dPID = 0.001;
    Servo intake2;

    /***************************/
/**** Shooting Stuff ***/
    /***************************/

    private int flickerState = 0;
    private boolean flickerActive = false;
    private long flickerTimer = 0;

    // Timing (milliseconds) — easy to tune
    long upTime = 900;
    long downTime = 800;

    public void startFlicker() {
        if (flickerActive) return;
        flickerActive = true;
        flickerState = 0;
        flickerTimer = System.currentTimeMillis();
    }


    public void updateFlicker() {
        if (!flickerActive) return;

        long now = System.currentTimeMillis();

        switch (flickerState) {

            case 0: // Slot 1 up
                slot1.setPosition(0.25);
                flickerTimer = now;
                flickerState++;
                break;

            case 1: // Slot 1 down
                if (now - flickerTimer >= upTime) {
                    slot1.setPosition(0.745);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 2: // Slot 2 up
                if (now - flickerTimer >= downTime) {
                    slot2.setPosition(0.60);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 3: // Slot 2 down
                if (now - flickerTimer >= upTime) {
                    slot2.setPosition(0.00);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 4: // Slot 3 up
                if (now - flickerTimer >= downTime) {
                    slot3.setPosition(0.60);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 5: // Slot 3 down
                if (now - flickerTimer >= upTime) {
                    slot3.setPosition(0.1);
                    flickerActive = false; // DONE
                }
                break;
        }
    }
    public boolean flickerDone() {
        return !flickerActive;
    }

    /***************************/
    /**** Normal ***/
    /***************************/

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),

                                    new Pose(135.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 8.000),

                                    new Pose(96.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }


    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new Meet3RedFarAuto.Paths(follower);
        follower.setStartingPose(new Pose(96, 8, Math.toRadians(0)));
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        intake2=hardwareMap.get(Servo.class, "intake2servo");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        turretmotor = hardwareMap.get(DcMotor.class, "turretmotor");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(0.4);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        follower.setMaxPower(0.95);
        lastTimeUpdated = System.nanoTime();

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        slot1.setPosition(0.745);
        slot2.setPosition(0.00);
        slot3.setPosition(0.1);



    }
    public void loop(){
        flywheel.setPower(0.62);
        intake.setPower(-1);
        intake2.setPosition(1);
        follower.update();
        updateFlicker();

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LLResult resultsofpooe = limelight.getLatestResult();
        boolean doesiseeitfoundboi = false;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
                double tx = fr.getTargetXDegrees();
                if(fr.getFiducialId()==24) {
                    derivativeTx = 1000000000.0*(tx-lastTx)/(System.nanoTime()-lastTimeUpdated);
                    turretmotor.setPower(pPID * tx + dPID * derivativeTx); //TxValue
                    doesiseeitfoundboi = true;
                    lastTx = tx;
                    lastTimeUpdated = System.nanoTime();
                    break;
                }
            }
        }
        if (!doesiseeitfoundboi) {
            turretmotor.setPower(0);
            telemetry.addData("Limelight", "No data available");
        }
    }
    public void autonomousPathUpdate() throws InterruptedException {

        switch (pathState) {
            //DO the state machine cayden
        }
    }

}