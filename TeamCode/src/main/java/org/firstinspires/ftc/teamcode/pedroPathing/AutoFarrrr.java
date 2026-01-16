package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
//@Autonomous(name="Meet 2 Red Autonomous")
public class AutoFarrrr extends OpMode {

    // Optional follower (pose/drawing only)
    public static Follower follower;

    // Timers
    private final Timer chargeTimer = new Timer();
    private final ElapsedTime servoTimer = new ElapsedTime();

    private final Timer initialMoveTimer = new Timer();
    private final Timer finalStrafeTimer = new Timer();

    // Main states: -1 = initial back, 0 = shoot, 1 = final strafe, 2 = done
    private int state = -1;

    // Shot sub-states: 0 = idle, 1 = up-wait, 2 = down-wait
    private int servoState = 0;
    private int shotsFired = 0;

    // Hardware
    private DcMotor flywheel;
    private DcMotor lf, rf, lr, rr;

    private Servo lift1, lift2, lift3;

    // Flywheel / timing
    private static final double FLYWHEEL_POWER = -0.70;
    private static final double CHARGE_TIME    = 4.0;

    private static final double LIFT_UP_WAIT   = 1.0;
    private static final double LIFT_DOWN_WAIT = 2.0;

    private static final int MAX_SHOTS = 3;

    // Initial move (backward)
    private static final double INITIAL_BACK_POWER = -0.4;
    private static final double INITIAL_BACK_TIME  = 1.1;

    // Final move (strafe)
    private static final double FINAL_STRAFE_POWER = 0.4;   // + = strafe right (usually)
    private static final double FINAL_STRAFE_TIME  = 1.0;

    // ─── YOUR SERVO CONSTANTS (UNCHANGED) ───
    private static final double LIFT1_UP   = 0.30;
    private static final double LIFT1_DOWN = 0.745;

    private static final double LIFT2_UP   = 0.55;
    private static final double LIFT2_DOWN = 0.05;

    private static final double LIFT3_UP   = 0.55;
    private static final double LIFT3_DOWN = 0.10;

    @Override
    public void init() {
        // Optional follower init
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(137.5)));
        } catch (Exception ignored) {}

        // Hardware map
        flywheel = hardwareMap.get(DcMotor.class, "testemotor");

        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        lift3 = hardwareMap.get(Servo.class, "lift3");

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);

        // Initial safe states
        flywheel.setPower(0.0);
        setDrivePower(0.0);

        // Start all lifts DOWN
        lift1.setPosition(LIFT1_DOWN);
        lift2.setPosition(LIFT2_DOWN);
        lift3.setPosition(LIFT3_DOWN);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        chargeTimer.resetTimer();
        servoTimer.reset();

        initialMoveTimer.resetTimer();
        finalStrafeTimer.resetTimer();

        state = -1;
        servoState = 0;
        shotsFired = 0;
    }

    @Override
    public void loop() {
        if (follower != null) follower.update();
        autonomousUpdate();

        telemetry.addData("State", state);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Servo State", servoState);
        telemetry.addData("Charge (s)", chargeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void autonomousUpdate() {
        switch (state) {

            case -1:
                // Initial backward move BEFORE shooting
                if (initialMoveTimer.getElapsedTimeSeconds() < INITIAL_BACK_TIME) {
                    setDrivePower(INITIAL_BACK_POWER);
                } else {
                    setDrivePower(0.0);

                    // Start charging AFTER the move (recommended)
                    chargeTimer.resetTimer();

                    state = 0;
                }
                break;

            case 0:
                // Flywheel on while shooting
                flywheel.setPower(FLYWHEEL_POWER);

                boolean ready = (shotsFired != 0) ||
                        (chargeTimer.getElapsedTimeSeconds() >= CHARGE_TIME);

                if (shotsFired < MAX_SHOTS) {
                    updateShotSequence(ready);
                } else {
                    finalStrafeTimer.resetTimer();
                    state = 1;
                }
                break;

            case 1:
                // Final STRAFE move
                if (finalStrafeTimer.getElapsedTimeSeconds() < FINAL_STRAFE_TIME) {
                    setStrafePower(FINAL_STRAFE_POWER);
                } else {
                    setDrivePower(0.0);
                    flywheel.setPower(0.0);
                    state = 2;
                }
                break;

            case 2:
                // Done
                setDrivePower(0.0);
                flywheel.setPower(0.0);
                break;
        }
    }

    // Shot sequence: (active servo) UP → wait → DOWN → wait → next shot
    private void updateShotSequence(boolean ready) {
        double t = servoTimer.seconds();

        // Stay still while shooting
        setDrivePower(0.0);

        switch (servoState) {
            case 0:
                if (ready) {
                    moveActiveLiftUp();      // lift2 → lift3 → lift1
                    servoTimer.reset();
                    servoState = 1;
                }
                break;

            case 1:
                if (t >= LIFT_UP_WAIT) {
                    moveActiveLiftDown();
                    servoTimer.reset();
                    servoState = 2;
                }
                break;

            case 2:
                if (t >= LIFT_DOWN_WAIT) {
                    shotsFired++;
                    servoState = 0;
                    servoTimer.reset();
                }
                break;
        }
    }

    // Order wanted: lift2 → lift3 → lift1
    // shotsFired:    0        1        2
    private void moveActiveLiftUp() {
        if (shotsFired == 0)      lift3.setPosition(LIFT3_UP);
        else if (shotsFired == 1) lift2.setPosition(LIFT2_UP);
        else                      lift1.setPosition(LIFT1_UP);
    }

    private void moveActiveLiftDown() {
        if (shotsFired == 0)      lift3.setPosition(LIFT3_DOWN);
        else if (shotsFired == 1) lift2.setPosition(LIFT2_DOWN);
        else                      lift1.setPosition(LIFT1_DOWN);
    }



    // Straight forward/back power
    private void setDrivePower(double p) {
        lf.setPower(p);
        rf.setPower(p);
        lr.setPower(p);
        rr.setPower(p);
    }

    // Mecanum strafe (positive = right for most configs; flip sign if it goes the wrong way)
    private void setStrafePower(double p) {
        lf.setPower(+p);
        rf.setPower(-p);
        lr.setPower(-p);
        rr.setPower(+p);
    }
}
