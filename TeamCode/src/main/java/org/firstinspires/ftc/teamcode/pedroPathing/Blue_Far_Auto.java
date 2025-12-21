package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Blue Far Auto")
public class Blue_Far_Auto extends OpMode {

    // Pedro follower
    public static Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;         // main auto state (0 = first path & shooting, 1 = final path, 2 = done)
    private Paths paths;

    // Flywheel + lift servo
    private DcMotor flywheel;
    private Servo lift;

    // Drivetrain motors for jerk (change names to match your config)
    private DcMotor lf, rf, lr, rr;

    // Charging timer (for flywheel)
    private Timer chargeTimer = new Timer();

    // Servo + jerk shot state machine
    private int servoState = 0;        // 0 idle/ready, 1 up-wait, 2 down-wait, 3 jerk F, 4 jerk B, 5 pause
    private Timer servoTimer = new Timer();
    private int shotsFired = 0;        // how many shots (up+down+jerk cycles) we've done

    // Path started flags so we don't restart them
    private boolean path1Started = false;
    private boolean path2Started = false;

    // Tunable constants
    private static final double FLYWHEEL_POWER = -0.53;   // flywheel spin power
    private static final double CHARGE_TIME   = 4.0;     // seconds to "charge" flywheel before first shot

    private static final double LIFT_UP_POS   = 0.325;
    private static final double LIFT_DOWN_POS = 0.685;

    private static final double LIFT_UP_WAIT   = 0.8;    // time to hold lift up
    private static final double LIFT_DOWN_WAIT = 0.8;    // time to hold lift down before jerk

    private static final double JERK_POWER     = 0.8;    // drive power during jerk
    private static final double JERK_TIME      = 0.30;   // forward/back jerk duration
    private static final double POST_JERK_PAUSE = 1;  // pause after jerk

    private static final int MAX_SHOTS = 3;              // do this many up-down-jerk cycles

    // ───────────────────────────────────────────────
    //  PATH DEFINITIONS
    // ───────────────────────────────────────────────
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21.000, 124.000),
                            new Pose(45.000, 101.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(45.000, 101.000),
                            new Pose(20.000, 90)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();
        }
    }

    // ───────────────────────────────────────────────
    //  INIT
    // ───────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(137.5)));

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // Map hardware
        flywheel = hardwareMap.get(DcMotor.class, "testemotor");
        lift = hardwareMap.get(Servo.class, "lift");

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        telemetry.addLine("Initialized Blue Far Auto!");
        telemetry.update();
        drawOnlyCurrent();
    }

    // ───────────────────────────────────────────────
    //  START
    // ───────────────────────────────────────────────
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        chargeTimer.resetTimer();
        servoTimer.resetTimer();

        pathState = 0;
        servoState = 0;
        shotsFired = 0;
        path1Started = false;
        path2Started = false;

        // Optionally set initial positions
        lift.setPosition(LIFT_DOWN_POS);
        flywheel.setPower(0);
    }

    // ───────────────────────────────────────────────
    //  LOOP
    // ───────────────────────────────────────────────
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Servo State", servoState);
        telemetry.addData("Charge Time", chargeTimer.getElapsedTimeSeconds());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        draw();
    }

    // Drawing helpers
    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    // ───────────────────────────────────────────────
    //  MAIN AUTO STATE MACHINE
    // ───────────────────────────────────────────────
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start Path1 once: "move back while charging motor"
                if (!path1Started) {
                    follower.followPath(paths.Path1);
                    path1Started = true;
                    chargeTimer.resetTimer(); // start charging as we move
                }

                // Flywheel charging / running the whole time in this state
                flywheel.setPower(FLYWHEEL_POWER);

                // Condition to ALLOW starting shots:
                //  - flywheel has charged for CHARGE_TIME
                //  - robot has finished Path1 (not busy)
                boolean readyToShoot = (chargeTimer.getElapsedTimeSeconds() >= CHARGE_TIME)
                        && !follower.isBusy();

                // Run up-down-jerk sequence up to MAX_SHOTS times
                if (shotsFired < MAX_SHOTS) {
                    updateShotSequence(readyToShoot);
                } else {
                    // After all shots fired, go to final position (Path2) once
                    if (!path2Started) {
                        follower.followPath(paths.Path2, true);
                        path2Started = true;
                        setPathState(1);
                    }
                }
                break;

            case 1:
                // Wait for Path2 to finish, then end auto
                if (!follower.isBusy()) {
                    flywheel.setPower(0.0);
                    setPathState(2); // done
                }
                break;
            case 2:
                // Auto finished; you could park, stop mechanisms, etc.
                flywheel.setPower(0.0);
                setDrivePower(0.0);
                break;
        }
    }

    // ───────────────────────────────────────────────
    //  SHOT SEQUENCE: up → wait → down → wait → jerk F/B
    //  Repeated MAX_SHOTS times
    // ───────────────────────────────────────────────
    private void updateShotSequence(boolean readyToStartShot) {
        double t = servoTimer.getElapsedTimeSeconds();

        switch (servoState) {
            case 0:
                // Idle, waiting for permission to start a new shot
                if (readyToStartShot) {
                    // Start new shot
                    lift.setPosition(LIFT_UP_POS);  // go UP
                    servoTimer.resetTimer();
                    servoState = 1;
                }
                break;

            case 1:
                // Hold UP for LIFT_UP_WAIT seconds
                lift.setPosition(LIFT_UP_POS);
                if (t >= LIFT_UP_WAIT) {
                    lift.setPosition(LIFT_DOWN_POS); // go DOWN
                    servoTimer.resetTimer();
                    servoState = 2;
                }
                break;

            case 2:
                // Hold DOWN for LIFT_DOWN_WAIT seconds before jerk
                lift.setPosition(LIFT_DOWN_POS);
                if (t >= LIFT_DOWN_WAIT) {
                    servoTimer.resetTimer();
                    servoState = 3;  // start jerk forward
                }
                break;

            case 3:
                // Jerk FORWARD
                lift.setPosition(LIFT_DOWN_POS);
                setDrivePower(JERK_POWER);
                if (t >= JERK_TIME) {
                    servoTimer.resetTimer();
                    servoState = 4; // then jerk backward
                }
                break;

            case 4:
                // Jerk BACKWARD
                lift.setPosition(LIFT_DOWN_POS);
                setDrivePower(-JERK_POWER);
                if (t >= JERK_TIME) {
                    setDrivePower(0.0);
                    servoTimer.resetTimer();
                    servoState = 5; // short pause after jerk
                }
                break;

            case 5:
                // Pause after jerk, keep servo DOWN
                lift.setPosition(LIFT_DOWN_POS);
                setDrivePower(0.0);
                if (t >= POST_JERK_PAUSE) {
                    // One full up–down–jerk cycle complete
                    shotsFired++;
                    servoState = 0;           // ready for next shot
                    servoTimer.resetTimer();
                }
                break;
        }
    }

    // Helper to set straight drive power (for jerk)
    private void setDrivePower(double power) {
        lf.setPower(power);
        rf.setPower(power);
        lr.setPower(power);
        rr.setPower(power);
    }

    // Utility — reset path state timer
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
