package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoFarrrrr")
public class AutoFarrrr extends OpMode {

    // (Optional) Pedro follower just for pose/drawing – not used for movement
    public static Follower follower;

    private Timer opmodeTimer;
    private Timer chargeTimer = new Timer();
    private Timer servoTimer = new Timer();
    private Timer finalDriveTimer = new Timer();

    // State machine
    private int state = 0;          // 0 = charge + cycles, 1 = final move, 2 = done

    // Flywheel + lift servo
    private DcMotor flywheel;
    private Servo lift;

    // Drivetrain motors for jerk + final move
    private DcMotor lf, rf, lr, rr;

    // Shot sequence state machine
    // 0 idle/ready, 1 up-wait, 2 down-wait, 3 jerk F, 4 jerk B, 5 pause
    private int servoState = 0;
    private int shotsFired = 0;

    // Tunable constants
    private static final double FLYWHEEL_POWER   = -0.85;  // flywheel spin power
    private static final double CHARGE_TIME      = 4.0;    // seconds to "charge" flywheel before first shot

    private static final double LIFT_UP_POS      = 0.325;
    private static final double LIFT_DOWN_POS    = 0.685;

    private static final double LIFT_UP_WAIT     = 0.8;    // time to hold lift up
    private static final double LIFT_DOWN_WAIT   = 0.8;    // time to hold lift down before jerk

    private static final double JERK_POWER       = 0.8;    // drive power during jerk
    private static final double JERK_TIME        = 0.30;   // forward/back jerk duration
    private static final double POST_JERK_PAUSE  = 1.0;    // pause after jerk

    private static final int    MAX_SHOTS        = 3;      // do this many up-down-jerk cycles

    // Final move constants
    private static final double FINAL_DRIVE_POWER = 0.4;   // slow forward move
    private static final double FINAL_DRIVE_TIME  = 1.0;   // how long to move forward (seconds)

    // ───────────────────────────────────────────────
    //  INIT
    // ───────────────────────────────────────────────
    @Override
    public void init() {

        // If you have Constants.createFollower, you can keep this for pose/drawing
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(137.5)));
        } catch (Exception e) {
            // If follower/drawing isn't set up, you can safely ignore or log
            telemetry.addLine("Follower init failed (ok for simple auto)");
        }

        opmodeTimer = new Timer();

        // Map hardware
        flywheel = hardwareMap.get(DcMotor.class, "testemotor");
        lift     = hardwareMap.get(Servo.class, "lift");

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        // Initial positions
        lift.setPosition(LIFT_DOWN_POS);
        flywheel.setPower(0.0);
        setDrivePower(0.0);

        telemetry.addLine("Initialized Blue Cycle In Place!");
        telemetry.update();
        drawOnlyCurrent();
    }

    // ───────────────────────────────────────────────
    //  START
    // ───────────────────────────────────────────────
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        chargeTimer.resetTimer();
        servoTimer.resetTimer();
        finalDriveTimer.resetTimer();

        state = 0;
        servoState = 0;
        shotsFired = 0;
    }

    // ───────────────────────────────────────────────
    //  LOOP
    // ───────────────────────────────────────────────
    @Override
    public void loop() {
        if (follower != null) follower.update(); // does nothing movement-wise since we never follow paths

        autonomousUpdate();

        telemetry.addData("State", state);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Servo State", servoState);
        telemetry.addData("Charge Time", chargeTimer.getElapsedTimeSeconds());
        if (follower != null) {
            telemetry.addData("Pose X", follower.getPose().getX());
            telemetry.addData("Pose Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        }
        telemetry.update();

        draw();
    }

    // Drawing helpers
    public static void drawOnlyCurrent() {
        try {
            if (follower != null) {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            }
        } catch (Exception e) {
            // ignore drawing errors
        }
    }

    public static void draw() {
        try {
            if (follower != null) Drawing.drawDebug(follower);
        } catch (Exception e) {
            // ignore drawing errors
        }
    }

    // ───────────────────────────────────────────────
    //  MAIN STATE MACHINE
    // ───────────────────────────────────────────────
    private void autonomousUpdate() {
        switch (state) {
            case 0:
                // Stay in place, charge flywheel, and perform shoot cycles

                // 1) Flywheel runs the whole time in this state
                flywheel.setPower(FLYWHEEL_POWER);

                // 2) Charge timer: wait CHARGE_TIME sec before first shot
                double chargeT = chargeTimer.getElapsedTimeSeconds();
                boolean readyToShootNow;

                if (shotsFired == 0) {
                    // First shot: require full charge
                    readyToShootNow = (chargeT >= CHARGE_TIME);
                } else {
                    // Later shots: no extra charge delay
                    readyToShootNow = true;
                }

                // 3) Run shot sequence up to MAX_SHOTS times
                if (shotsFired < MAX_SHOTS) {
                    updateShotSequence(readyToShootNow);
                } else {
                    // 4) After all shots, transition to final move state
                    finalDriveTimer.resetTimer();
                    state = 1;
                }
                break;

            case 1:
                // Final forward move
                double moveT = finalDriveTimer.getElapsedTimeSeconds();
                if (moveT < FINAL_DRIVE_TIME) {
                    setDrivePower(FINAL_DRIVE_POWER);  // move forward
                } else {
                    setDrivePower(0.0);
                    flywheel.setPower(0.0);
                    state = 2;  // done
                }
                break;

            case 2:
                // Done — robot just holds still
                setDrivePower(0.0);
                flywheel.setPower(0.0);
                lift.setPosition(LIFT_DOWN_POS);
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
                    // Start new shot: go UP
                    lift.setPosition(LIFT_UP_POS);
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

    // Helper to set straight drive power (for jerk + final move)
    private void setDrivePower(double power) {
        lf.setPower(power);
        rf.setPower(power);
        lr.setPower(power);
        rr.setPower(power);
    }
}
