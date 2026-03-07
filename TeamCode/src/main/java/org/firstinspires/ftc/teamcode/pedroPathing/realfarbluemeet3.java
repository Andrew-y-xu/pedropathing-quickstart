package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
@Autonomous(name="Far Blue")
public class realfarbluemeet3 extends OpMode {
    DcMotor flywheel;
    DcMotor flywheel2;

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
    double pPID = 0.007; //0.11 --> 0.04 (original value)
    double dPID = 0.003; //0.003 --> 0.001 (original value)
    double iPID = 0;
    Servo intake2;
//    double currenttime;
//    ElapsedTime timer=new ElapsedTime();

    // Intake pulse state
    private boolean intakePulseActive = false;
    private long intakePulseStart = 0;
    private boolean intakePulseEnabled = false;

    // tune this
    private long intakePulseTime = 600; // ms
    public void startIntakePulse() {
        if (!intakePulseEnabled) return;
        if (intakePulseActive) return;

        intakePulseActive = true;
        intakePulseStart = System.currentTimeMillis();
        intake.setPower(1);
        intake2.setPosition(0);
    }

    public void updateIntakePulse() {
        if (!intakePulseActive) return;

        if (System.currentTimeMillis() - intakePulseStart >= intakePulseTime) {
            intake.setPower(-1);     // return to normal intake
            intake2.setPosition(1);
            intakePulseActive = false;
        }
    }

    // Startup delay for flywheel spin-up
    private boolean startupDelayActive = true;
    private long startupDelayStart = 0;
    private long startupDelayTime = 6000; // ms (tune this)

    /***************************/
/**** Shooting Stuff ***/
    /***************************/

    private int flickerState = 0;
    private boolean flickerActive = false;
    private long flickerTimer = 0;

    // Timing (milliseconds) — easy to tune
    long upTime = 300;
    long downTime = 2000;

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
                slot1.setPosition(0.47);
                flickerTimer = now;
                flickerState++;
                break;

            case 1: // Slot 1 down
                if (now - flickerTimer >= upTime) {
                    slot1.setPosition(0.98);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 2: // Slot 2 up
                if (now - flickerTimer >= downTime) {
                    slot2.setPosition(0.58);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 3: // Slot 2 down
                if (now - flickerTimer >= upTime) {
                    slot2.setPosition(0.05);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 4: // Slot 3 up
                if (now - flickerTimer >= downTime) {
                    slot3.setPosition(0.63);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 5: // Slot 3 down
                if (now - flickerTimer >= upTime) {
                    slot3.setPosition(0.12);
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
        public PathChain Path3;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 8.000),

                                    new Pose(8.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.000, 8.000),

                                    new Pose(48.000, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.000, 8.000),
                                    new Pose(24.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }


    enum IntakeMode {
        INTAKE,
        OUTTAKE,
        OFF
    }

    private IntakeMode intakeMode = IntakeMode.INTAKE;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new realfarbluemeet3.Paths(follower);
        follower.setStartingPose(new Pose(48, 8, Math.toRadians(180)));
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        intake2=hardwareMap.get(Servo.class, "intake2servo");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        turretmotor = hardwareMap.get(DcMotor.class, "turretmotor");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        flywheel2=hardwareMap.get(DcMotor.class,"flywheelmotor2");

        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(0.8);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        follower.setMaxPower(0.95);
        lastTimeUpdated = System.nanoTime();
//        startupDelayStart = System.currentTimeMillis();
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        slot1.setPosition(0.98);
        slot2.setPosition(0.05);
        slot3.setPosition(0.12);



    }
    public void start(){
        startupDelayStart=System.currentTimeMillis();
    }
    public void loop(){
        follower.update();
        flywheel.setPower(0.63);
        flywheel2.setPower(0.63);

        updateFlicker();
        //updateIntakePulse();

        if (intakePulseActive) {
            // Intake pulse is controlling intake right now.
            // Do nothing here so it doesn't get overridden.
        } else {
            switch (intakeMode) {
                case INTAKE:
                    intake.setPower(-1);
                    intake2.setPosition(1);
                    break;

                case OUTTAKE:
                    intake.setPower(1);
                    intake2.setPosition(0);
                    break;

                case OFF:
                    intake.setPower(0);
                    intake2.setPosition(0.5);
                    break;
            }
        }



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
                if(fr.getFiducialId()>0) {
                    derivativeTx = 1000000000.0*(tx-lastTx)/(System.nanoTime()-lastTimeUpdated);
                    turretmotor.setPower(pPID * tx + dPID * derivativeTx); //TxValue
                    doesiseeitfoundboi = true;
                    lastTx = tx;
                    lastTimeUpdated = System.nanoTime();
                    telemetry.addData("Limelight","Data available");
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

            // ---- Go to hub (ends at shared point), then flick, then leave ----
            case 0:
                // Let flywheel spin up before flicker

                if (System.currentTimeMillis() - startupDelayStart >= startupDelayTime) {
                    startFlicker();
                    pathState = 101;
                }
                break;


            case 101: // wait for Path1 to finish (now standing still at hub)
                if (flickerDone()) {
                    intakeMode = realfarbluemeet3.IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path1, true);
                        pathState = 2;
                }
                break;

            // ---- Normal travel steps (no flicker here) ----
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path2, true);
                    pathState = 4;
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
                if (!follower.isBusy()) { // standing still at hub
//                    currenttime=timer.seconds();
//                    if(timer.seconds()-4>currenttime) {
                        startFlicker();
                        pathState = 401;
                }
                break;

            case 401:
                if (flickerDone()) {
                    follower.followPath(paths.Path3,true);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    flywheel.setPower(0);
                    intake.setPower(0);
                    flywheel2.setPower(0);
                    intake2.setPosition(0.5);
                    pathState=9;
                }

            case 9:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }

}