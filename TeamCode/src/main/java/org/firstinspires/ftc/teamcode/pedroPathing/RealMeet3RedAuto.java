package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
public class RealMeet3RedAuto extends OpMode {
    DcMotor flywheel;
    Servo slot1;
    Servo slot2;
    Servo slot3;
    DcMotor intake;
    private Follower follower;
    double derivativeTx = 0;
    private int pathState=0;
    private Meet3RedAuto.Paths paths;
    private Limelight3A limelight;
    DcMotor turretmotor;
    double lastTx = 0;
    double lastTimeUpdated = 0;
    double pPID = 0.04;
    double dPID = 0.001;
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.500, 122.000),

                                    new Pose(84.000, 83.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(35.5))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 83.500),

                                    new Pose(125.000, 83.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 83.500),

                                    new Pose(84.000, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35.5))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 83.500),
                                    new Pose(96.000, 51.000),
                                    new Pose(125.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 60.000),

                                    new Pose(84.000, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35.5))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 83.500),
                                    new Pose(84.000, 27.000),
                                    new Pose(125.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 35.000),

                                    new Pose(84.000, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35.5))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 83.500),

                                    new Pose(124.000, 69.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(270))

                    .build();
        }
    }
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new Meet3RedAuto.Paths(follower);
        follower.setStartingPose(new Pose(123.5, 122, Math.toRadians(35.5)));
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        turretmotor = hardwareMap.get(DcMotor.class, "turretmotor");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        flywheel.setPower(-1);//will change soon
        intake.setPower(-1);
        //MAKE THE INTAKE SERVO SPIN OR WHATEVER
    }
    public void loop(){
        follower.update();
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
            case 0:
                follower.followPath(paths.Path1);
                pathState=1;
                break;

            case 1:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState=2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState=3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState=4;
                }
                break;

            case 4:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState=5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState=6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState=7;
                }
                break;

            case 7:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState=8;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    pathState=-1;
                }
                break;
        }
    }
    public void shooterUpdates(){
        //WRITE THE FLIPPERS HERE
        //USE SLOT1, SLOT2 and SLOT3 from above
    }
}
