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

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Meet3BlueAuto extends OpMode{
    Map<Integer, Character> indexer = new HashMap<>();
    int shots=0;
    int randomizationState;
    Timer timer;
    DcMotor flywheel;
    Servo[] slot;
    int shooterState;
    String order;
    DcMotor intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Paths paths;
    double derivativeTx = 0;
    private Limelight3A lookylookyseesee;
    DcMotor poopeemotorey;
    double lastTx = 0;
    double lastTimeUpdated = 0;
    double pPID = 0.04;
    double dPID = 0.001;
    //    private NormalizedColorSensor color1;
    private NormalizedColorSensor color2;
    private NormalizedColorSensor color3;
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20.500, 122.000),
                            new Pose(38.500, 104.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(144.5),
                            Math.toRadians(144.5))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(38.500, 104.000),
                            new Pose(62.500, 85.000),
                            new Pose(42.500, 83.500)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(144.5),
                            Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.500, 83.500),
                            new Pose(19.000, 83.500)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(19.000, 83.500),
                            new Pose(38.500, 104.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(144.5))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(38.500, 104.000),
                            new Pose(63.000, 88.000),
                            new Pose(66.000, 62.000),
                            new Pose(42.500, 60.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(144.5),
                            Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.500, 60.000),
                            new Pose(19.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(19.000, 60.000),
                            new Pose(44.500, 70.000),
                            new Pose(38.500, 104.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(144.5))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(38.500, 104.000),
                            new Pose(65.000, 85.000),
                            new Pose(65.000, 31.000),
                            new Pose(42.500, 35.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(144.5),
                            Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.500, 35.000),
                            new Pose(19.000, 35.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(19.000, 35.000),
                            new Pose(70.000, 44.000),
                            new Pose(38.500, 104.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(144.5))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(38.500, 104.000),
                            new Pose(80.000, 20.000),
                            new Pose(9.000, 9.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(144.5),
                            Math.toRadians(180))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20.000, 9.000),
                            new Pose(9.000, 9.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000, 9.000),
                            new Pose(38.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(144.5))
                    .build();
        }
    }
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new Meet3BlueAuto.Paths(follower);
        follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(144.5)));
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        timer = new Timer();
        intake=hardwareMap.get(DcMotor.class,"intakemotor");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        indexer.put(0,'n');
        indexer.put(1,'n');
        indexer.put(2,'n');
        slot = new Servo[3];
        slot[0]=hardwareMap.get(Servo.class,"lift1");
        slot[1]=hardwareMap.get(Servo.class,"lift2");
        slot[2]=hardwareMap.get(Servo.class,"lift3");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
        poopeemotorey = hardwareMap.get(DcMotor.class, "turretmotor");
        lookylookyseesee = hardwareMap.get(Limelight3A.class, "limelight");
        lookylookyseesee.pipelineSwitch(0);
        lookylookyseesee.start();
//        color1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        color3 = hardwareMap.get(NormalizedColorSensor.class, "color3");
        flywheel.setPower(-1);
//        intake.setPower(-1);
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = 0; // start at first path
    }
    @Override
    public void loop(){
        follower.update();          // must always run for movement
        try {
            autonomousPathUpdate();     // handles path transitions
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LLResult resultsofpooe = lookylookyseesee.getLatestResult();
        boolean doesiseeitfoundboi = false;

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = resultsofpooe.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
//                Double TxValue = resultsofpooe.getTx();
                double tx = fr.getTargetXDegrees();
                if(fr.getFiducialId()==24) {
                    derivativeTx = 1000000000.0*(tx-lastTx)/(System.nanoTime()-lastTimeUpdated);
                    poopeemotorey.setPower(pPID * tx + dPID * derivativeTx); //TxValue
                    doesiseeitfoundboi = true;
                    lastTx = tx;
                    lastTimeUpdated = System.nanoTime();
                    break;
                }
            }

        }
        if (!doesiseeitfoundboi) {
            poopeemotorey.setPower(0);
            telemetry.addData("Limelight", "No data available");
        }
    }
    public void shooterUpdates() throws InterruptedException {
        switch(shooterState){
            case 0:
                setIndexer(indexer);
                if(shots!=0){
                    setState(1);
                    break;
                }
                else{
                    setState(0);
                    break;
                }
            case 1:
                order=spinUp(indexer,flywheel);
                setState(2);
            case 2:
                int first= Integer.parseInt(String.valueOf(order.charAt(0)));
                int second= Integer.parseInt(String.valueOf(order.charAt(1)));
                int third= Integer.parseInt(String.valueOf(order.charAt(2)));
                if(indexer.get(first)!='n'){
                    slot[first].setPosition(servopositionup(first));
                    shots--;
                    Thread.sleep(200);
                    slot[first].setPosition(servopositionhalf(first));
                }
                if(indexer.get(second)!='n'){
                    slot[second].setPosition(servopositionup(second));
                    shots--;
                    Thread.sleep(200);
                    slot[second].setPosition(servopositionhalf(second));
                }
                if(indexer.get(third)!='n'){
                    slot[third].setPosition(servopositionup(third));
                    shots--;
                    Thread.sleep(200);
                    slot[third].setPosition(servopositionhalf(third));
                }
                if(shots>0){
                    setState(0);
                }
                slot[0].setPosition(0.745);
                slot[1].setPosition(0.05);
                slot[2].setPosition(0.1);
        }
    }
    public void setIndexer(Map<Integer, Character> indexer) {
        this.indexer = indexer;
//        indexer.put(0,detectColor(color1));
//        if(detectColor(color1)!='n'){
//            shots++;
//        }
        indexer.put(1,detectColor(color2));
        if(detectColor(color2)!='n'){
            shots++;
        }
        indexer.put(2,detectColor(color3));
        if(detectColor(color3)!='n'){
            shots++;
        }
        if(detectColor(color2)=='p'&&detectColor(color3)=='p'){
            indexer.put(0,'g');
        }
        else{
            indexer.put(0,'p');
        }
    }
    public String spinUp(Map<Integer, Character> indexer,DcMotor flywheel) {
        this.indexer = indexer;
        int keygreen = getKeyFromValueStream(indexer, 'g');
        switch (randomizationState) {
            case 0:
                //GPP
                if (keygreen == 0) {
                    return "012";
                }
                else if (keygreen == 1) {
                    return "102";
                }
                else if (keygreen == 2) {
                    return "201";
                }
                break;
            case 1:
                if (keygreen == 0) {
                    return "102";
                }
                if (keygreen == 1) {
                    return "012";
                }
                if (keygreen == 2) {
                    return "120";
                }
                //PGP
                break;
            case 2:
                if (keygreen == 0) {
                    return "120";
                }
                if (keygreen == 1) {
                    return "021";
                }
                if (keygreen == 2) {
                    return "012";
                }
                //PPG
                break;
        }
        return "012";
    }
    private void setState(int newState) {
        shooterState=newState;
        timer.resetTimer();
    }
    public static <K, V> int getKeyFromValueStream(Map<K, V> map, V value) {
        return map.entrySet().stream()
                .filter(entry -> Objects.equals(entry.getValue(), value))
                .map(Map.Entry::getKey)
                .findFirst()
                .map(key -> (Integer) key) // assuming the key is Integer
                .orElse(-1); // default value if no key is found
    }
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;

//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path4, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                shooterUpdates();
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5, true);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path6, true);
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path7, true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                shooterUpdates();
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path8, true);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path9, true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path10, true);
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                shooterUpdates();
//                if(!follower.isBusy()) {
//                    follower.followPath(paths.Path11, true);
//                    setPathState(11);
//                }
//                break;
//
//            case 11:
//                if(!follower.isBusy()) {
//                    follower.followPath(paths.Path12, true);
//                    setPathState(12);
//                }
//                break;
//
//            case 12:
//                if(!follower.isBusy()) {
//                    follower.followPath(paths.Path13,true);
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                if (!follower.isBusy()) {
//                    setPathState(-1); // finished
//                }
//                break;
       }
    }
    private double clamp01(float v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
    private double servopositionup(int which){
        if(which==0){
            return 0.3;
        }
        else if(which==1){
            return 0.55;
        }
        else{
            return 0.55;
        }
    }
    private double servopositionhalf(int which){
        if(which==0){
            return 0.5225;
        }
        else if(which==1){
            return 0.3;
        }
        else{
            return 0.325;
        }
    }
    private char detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double r = clamp01(rgba.red);
        double g = clamp01(rgba.green);
        double b = clamp01(rgba.blue);

        double intensity = r + g + b;
        boolean brightEnough = intensity > 0.06;
        boolean notBlownOut = intensity < 2.7;

        double sum = Math.max(1e-6, intensity);
        double rn = r / sum;
        double gn = g / sum;
        double bn = b / sum;

        boolean isGreen =
                brightEnough && notBlownOut &&
                        gn > 0.45 &&
                        gn > rn + 0.05 &&
                        gn > bn + 0.05;

        boolean isPurple =
                brightEnough && notBlownOut &&
                        rn > 0.20 &&
                        bn > 0.40 &&
                        gn < 0.35 &&
                        Math.abs(rn - bn) < 0.30;

        if (isGreen) {
            return 'g';
        } else if (isPurple) {
            return 'p';
        } else {
            return 'n';
        }
    }
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}