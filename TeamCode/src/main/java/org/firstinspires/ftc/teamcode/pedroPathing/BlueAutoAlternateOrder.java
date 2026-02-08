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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
@Autonomous(name="BlueAutoRLB")
public class BlueAutoAlternateOrder extends OpMode {
    DcMotor flywheel;
    Servo hoodservo;
    Servo slot1;
    Servo slot2;
    Servo slot3;
    private long delayStart1 = 0;
    private long delayStart2 = 0;
    private long delayStart3 = 0;
    private boolean delay1Active = false;
    private boolean delay2Active = false;
    private boolean delay3Active = false;
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
                slot1.setPosition(0.2);

                flickerTimer = now;
                flickerState++;
                break;

            case 1: // Slot 1 down
                if (now - flickerTimer >= upTime) {

                    slot1.setPosition(0.9);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 2: // Slot 2 up
                if (now - flickerTimer >= downTime) {

                    slot2.setPosition(0.65);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 3: // Slot 2 down
                if (now - flickerTimer >= upTime) {

                    slot2.setPosition(0);
                    flickerTimer = now;
                    flickerState++;
                }
                break;

            case 4: // Slot 3 up
                if (now - flickerTimer >= downTime) {
                    slot3.setPosition(0.65);
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
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.5, 127.000), new Pose(58.000, 83.500)))
                    .setConstantHeadingInterpolation(Math.toRadians(144.5))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.000, 83.500), new Pose(18.000, 83.500)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18, 83.5), new Pose(58.000, 83.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58.000, 83.500), new Pose(48.000, 51.000), new Pose(11.5, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.5000, 60.000), new Pose(58.000, 83.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58, 83.500), new Pose(60.000, 27.000), new Pose(11.500, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.500, 35.000), new Pose(58.000, 105.00)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    enum IntakeMode {
        INTAKE,
        OUTTAKE,
        OFF
    }
    long flywheelStartTime;
    boolean flywheelSpunUp = false;
    long flywheelWaitStart = 0;
    boolean flywheelReady = false;
    boolean flywheelTimerStarted = false;
    private IntakeMode intakeMode = IntakeMode.INTAKE;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new BlueAutoAlternateOrder.Paths(follower);
        follower.setStartingPose(new Pose(20.5, 127, Math.toRadians(144.5)));
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        intake2=hardwareMap.get(Servo.class, "intake2servo");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        turretmotor = hardwareMap.get(DcMotor.class, "turretmotor");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(0.68);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        follower.setMaxPower(0.95);
        lastTimeUpdated = System.nanoTime();

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        slot1.setPosition(0.9);
        slot2.setPosition(0.00);
        slot3.setPosition(0.1);



    }
    public void loop(){
        follower.update();
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
                if(fr.getFiducialId() ==20) {
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
        } else {
            telemetry.addData("Limelight","Data available");
        }
        telemetry.update();
    }
    public void autonomousPathUpdate() throws InterruptedException {

        switch (pathState) {

            // ---- Go to hub (ends at shared point), then flick, then leave ----
            case 0:
                flywheel.setPower(0.6);
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {

                    if (!flywheelTimerStarted) {
                        flywheelWaitStart = System.currentTimeMillis();
                        flywheelTimerStarted = true;
                    }

                    if (System.currentTimeMillis() - flywheelWaitStart >= 1000) {
                        flywheelReady = true;
                        startFlicker();
                        pathState = 101;
                    }
                }
                break;

            case 101: // wait for flicker to finish, then go to Path2
                if (flickerDone()) {
                    flywheel.setPower(0.48);
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path2, true);
                    pathState = 2;
                }
                break;

            // ---- Normal travel steps (no flicker here) ----
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path3, true);
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
                    if (!delay1Active) {
                        delayStart1 = System.currentTimeMillis();
                        delay1Active = true;
                    }
                    if (delay1Active && (System.currentTimeMillis() - delayStart1 >= 300)) {
                        startFlicker();
                        pathState = 401;
                    }
                }
                break;

            case 401:
                if (flickerDone()) {
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path4, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    //startIntakePulse();
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path5, true); // ends at hub
                    pathState = 6;
                }
                break;

            // ---- Arrived at hub -> flick -> then Path7 ----
            case 6:
                if (!follower.isBusy()) { // standing still at hub
                    if (!delay2Active) {
                        delayStart2 = System.currentTimeMillis();
                        delay2Active = true;
                    }
                    if (delay2Active && (System.currentTimeMillis() - delayStart2 >= 300)) {
                        startFlicker();
                        pathState = 601;
                    }
                }
                break;

            case 601:
                if (flickerDone()) {
                    intakeMode = IntakeMode.INTAKE;
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.Path6, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    //startIntakePulse();
                    follower.setMaxPower(1.00);
                    follower.followPath(paths.Path7, true); // ends at hub
                    pathState = 8;
                }
                break;

            // ---- Arrived at hub -> flick -> then Path9 ----
            case 8:
                if (!follower.isBusy()) { // standing still at hub
                    if (!delay3Active) {
                        delayStart3 = System.currentTimeMillis();
                        delay3Active = true;
                    }
                    if (delay3Active && (System.currentTimeMillis() - delayStart3 >= 300)) {
                        startFlicker();
                        pathState = 801;
                    }
                }
                break;

            case 801:
                if (flickerDone()) {
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }

}