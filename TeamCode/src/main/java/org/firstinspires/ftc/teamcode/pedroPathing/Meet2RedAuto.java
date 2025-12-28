package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class Meet2RedAuto extends OpMode{
    Map<Integer, Character> indexer = new HashMap<>();
    int shots=0;
    int randomizationState;
    Timer timer;
    DigitalChannel pin0;
    DigitalChannel pin1;
    DigitalChannel pin2;
    DigitalChannel pin3;
    DigitalChannel pin4;
    DigitalChannel pin5;
    DcMotor flywheel;
    Servo[] slot;
    int shooterState;
    String order;
    DcMotor intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Paths paths;
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.500, 122.000), new Pose(105.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(35.5))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(105.500, 104.000),
                                    new Pose(81.500, 85.000),
                                    new Pose(101.500, 83.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.500, 83.500), new Pose(125.000, 83.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.500), new Pose(105.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35.5))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(105.500, 104.000),
                                    new Pose(81.000, 88.000),
                                    new Pose(78.000, 62.000),
                                    new Pose(101.500, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.500, 60.000), new Pose(125.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.000, 60.000),
                                    new Pose(100.000, 70.000),
                                    new Pose(105.500, 104.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35.5))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(105.500, 104.000),
                                    new Pose(79.000, 85.000),
                                    new Pose(79.000, 31.000),
                                    new Pose(101.500, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35.5), Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.500, 35.000), new Pose(125.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.000, 35.000),
                                    new Pose(74.000, 44.000),
                                    new Pose(105.500, 104.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(105.500, 104.000),
                            new Pose(64.000, 20.000),
                            new Pose(135.000, 9.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(35.5),
                            Math.toRadians(0))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.000, 9.000),
                            new Pose(135.000, 9.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(135.000, 9.000),
                            new Pose(105.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(35.5))
                    .build();
        }
    }
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(137.5)));
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        intake=hardwareMap.get(DcMotor.class,"intake");
        telemetry.addLine("Initialized Blue Auto!");
        telemetry.update();
        indexer.put(0,'n');
        indexer.put(1,'n');
        indexer.put(2,'n');
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        pin2 = hardwareMap.digitalChannel.get("digital2");
        pin3 = hardwareMap.digitalChannel.get("digital3");
        pin4 = hardwareMap.digitalChannel.get("digital4");
        pin5 = hardwareMap.digitalChannel.get("digital5");
        flywheel=hardwareMap.get(DcMotor.class,"flywheel");
        slot[0]=hardwareMap.get(Servo.class,"slot0");
        slot[1]=hardwareMap.get(Servo.class,"slot1");
        slot[2]=hardwareMap.get(Servo.class,"slot2");
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
                    slot[first].setPosition(0.685);
                    shots--;
                    wait(200);
                    slot[first].setPosition(0.505);
                }
                if(indexer.get(second)!='n'){
                    slot[second].setPosition(0.685);
                    shots--;
                    wait(200);
                    slot[second].setPosition(0.505);
                }
                if(indexer.get(third)!='n'){
                    slot[third].setPosition(0.685);
                    shots--;
                    wait(200);
                    slot[third].setPosition(0.505);
                }
                if(shots>0){
                    setState(0);
                }
                flywheel.setPower(0);
                slot[first].setPosition(0.325);
                slot[second].setPosition(0.325);
                slot[third].setPosition(0.325);
        }
    }
    public void setIndexer(Map<Integer, Character> indexer) {
        this.indexer = indexer;
        if(pin0.getState()){
            indexer.put(0,'p');
            shots++;
        }
        else if(pin1.getState()){
            indexer.put(0,'g');
            shots++;
        }
        else{
            indexer.put(0,'n');
        }
        if(pin2.getState()){
            indexer.put(1,'p');
            shots++;
        }
        else if(pin3.getState()){
            indexer.put(1,'g');
            shots++;
        }
        else{
            indexer.put(1,'n');
        }
        if(pin4.getState()){
            indexer.put(2,'p');
            shots++;
        }
        if(pin5.getState()){
            indexer.put(2,'g');
            shots++;
        }
        else{
            indexer.put(2,'n');
        }
    }
    public String spinUp(Map<Integer, Character> indexer,DcMotor flywheel) {
        this.indexer = indexer;
        this.flywheel = flywheel;
        flywheel.setPower(1);
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

            case 2:
                intake.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                intake.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                intake.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                intake.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                shooterUpdates();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;

            case 8:
                intake.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;

            case 9:
                intake.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    setPathState(10);
                }
                break;

            case 10:
                shooterUpdates();
                if (!follower.isBusy()) {
                    setPathState(-1); // finished
                }
                break;
        }
    }
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}