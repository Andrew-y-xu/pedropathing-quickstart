package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.hardware.AutoShooter;
import org.firstinspires.ftc.teamcode.colorIndicator;

@TeleOp(name = "BlueTeleop_MosaicShoot")
public class TeleOpAutoShooter extends OpMode {

    // ================= HARDWARE =================
    DcMotor testmotor;       // flywheel (assumed)
    DcMotor intakemotor;
    DcMotor poopeemotorey;   // turret

    DcMotorEx motor_frontLeft;
    DcMotorEx motor_frontRight;
    DcMotorEx motor_backLeft;
    DcMotorEx motor_backRight;

    Servo liftservo;    // flicker BACK
    Servo lift2servo;   // flicker LEFT
    Servo lift3servo;   // flicker RIGHT
    Servo intake2servo;
    Servo hoodservo;

    Limelight3A lookylookyseesee;
    AutoShooter autoShoot = new AutoShooter();

    // ================= DRIVE =================
    double speed = 1.0;

    // ================= LIMELIGHT =================
    double limelight_tx = 0;
    double limelight_ty = 0;
    boolean targetVisible = false;

    // Turret PD variables
    double lastTx = 0;
    long lastTimeUpdated = System.nanoTime();
    double pPID = 0.01;   // tune
    double dPID = 0.0005; // tune

    // ================= MOSAIC SHOOT =================
    enum ShootState { IDLE, SHOT_1, SHOT_2, SHOT_3 }
    ShootState shootState = ShootState.IDLE;

    colorIndicator.Slot[] shootOrder = null;
    int shootIndex = 0;

    ElapsedTime shootTimer = new ElapsedTime();
    boolean shootButtonWasPressed = false;

    // ================= INIT =================
    @Override
    public void init() {

        testmotor = hardwareMap.dcMotor.get("testemotor");
        intakemotor = hardwareMap.dcMotor.get("intakemotor");
        poopeemotorey = hardwareMap.dcMotor.get("turretmotor");

        motor_frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        motor_frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        motor_backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        motor_backRight = hardwareMap.get(DcMotorEx.class, "rr");

        liftservo = hardwareMap.get(Servo.class, "lift2");
        lift2servo = hardwareMap.get(Servo.class, "lift1");
        lift3servo = hardwareMap.get(Servo.class, "lift3");
        intake2servo = hardwareMap.get(Servo.class, "intake2servo");
        hoodservo = hardwareMap.get(Servo.class, "hood");

        lookylookyseesee = hardwareMap.get(Limelight3A.class, "limelight");
        lookylookyseesee.pipelineSwitch(0);
        lookylookyseesee.start();

        // Directions
        testmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_frontLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_backRight.setDirection(DcMotor.Direction.REVERSE);
        motor_backLeft.setDirection(DcMotor.Direction.REVERSE);
        // poopeemotorey direction: set as needed depending on turret polarity
        // poopeemotorey.setDirection(DcMotorSimple.Direction.FORWARD);

        // Modes & behaviors (explicit for safety)
        motor_frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        poopeemotorey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        poopeemotorey.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initial servo positions (tune to your hardware)
        liftservo.setPosition(0.0);
        lift2servo.setPosition(0.745);
        lift3servo.setPosition(0.1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        // ---------- DRIVE ----------
        double vx = -gamepad1.left_stick_y * speed;
        double vy = gamepad1.left_stick_x * speed;
        double o  = gamepad1.right_stick_x * speed;

        motor_frontLeft.setPower(vx + vy + o);
        motor_frontRight.setPower(vx - vy - o);
        motor_backLeft.setPower(vx - vy + o);
        motor_backRight.setPower(vx + vy - o);

        // ---------- LIMELIGHT APRILTAG & TURRET ----------
        boolean doesiseeitfoundboi = false;
        LLResult resultsofpooe = lookylookyseesee.getLatestResult();

        if (resultsofpooe != null && resultsofpooe.isValid()) {
            for (LLResultTypes.FiducialResult fr : resultsofpooe.getFiducialResults()) {

                limelight_tx = fr.getTargetXDegrees();
                limelight_ty = fr.getTargetYDegrees();

                if (fr.getFiducialId() == 20) {
                    doesiseeitfoundboi = true;

                    // Turret PD control with derivative guard & power clamp
                    long now = System.nanoTime();
                    long dt = now - lastTimeUpdated;
                    if (dt > 0) {
                        double derivativeTx = 1e9 * (limelight_tx - lastTx) / dt; // deg/sec
                        double control = pPID * limelight_tx + dPID * derivativeTx;
                        control = Range.clip(control, -0.6, 0.6); // clamp to safe speed
                        poopeemotorey.setPower(control);
                    }
                    lastTx = limelight_tx;
                    lastTimeUpdated = now;

                    // Setup shoot order for mosaic (once we see the tag)
                    if (shootOrder == null) {
                        shootOrder = colorIndicator.computeOrder(
                                colorIndicator.ColorType.PURPLE,
                                colorIndicator.ColorType.PURPLE,
                                colorIndicator.OrderPolicy.PHYSICAL_LBR
                        );
                    }

                    break; // stop after first matching fiducial
                }
            }
        }

        targetVisible = doesiseeitfoundboi;
        telemetry.addData("Limelight Status", doesiseeitfoundboi ? "Data is available" : "No data available");

        // ---------- START SHOOT (ONE BUTTON) ----------
        boolean shootPressed = gamepad2.a;

        if (shootPressed && !shootButtonWasPressed
                && shootOrder != null
                && shootOrder.length >= 3
                && shootState == ShootState.IDLE
                && targetVisible) {

            shootState = ShootState.SHOT_1;
            shootIndex = 0;
            shootTimer.reset();
        }
        shootButtonWasPressed = shootPressed;

        // ---------- SHOOT SEQUENCE ----------
        if (shootState != ShootState.IDLE && shootOrder != null && shootIndex < shootOrder.length) {

            colorIndicator.Slot slot = shootOrder[shootIndex];
            Servo activeFlicker =
                    (slot == colorIndicator.Slot.LEFT)  ? lift2servo :
                            (slot == colorIndicator.Slot.BACK)  ? liftservo  :
                                    (slot == colorIndicator.Slot.RIGHT) ? lift3servo : null;

            if (activeFlicker != null) {
                double t = shootTimer.milliseconds();

                if (t < 250) {
                    activeFlicker.setPosition(0.60); // kick up
                } else if (t < 450) {
                    activeFlicker.setPosition(0.00); // back down
                } else {
                    shootIndex++;
                    shootTimer.reset();

                    if      (shootIndex == 1) shootState = ShootState.SHOT_2;
                    else if (shootIndex == 2) shootState = ShootState.SHOT_3;
                    else                      shootState = ShootState.IDLE;
                }
            } else {
                // Unknown slot: abort sequence safely
                shootState = ShootState.IDLE;
            }
        }

        // ---------- AUTO SHOOTER ----------
        if (targetVisible) {
            autoShoot.advancedMathematics(limelight_ty);
            testmotor.setPower(autoShoot.getFlywheelPower());
            hoodservo.setPosition(autoShoot.getAnglePosition());
        } else {
            testmotor.setPower(0);
            // Optionally park hood: hoodservo.setPosition( /* your parked pos */ );
        }

        // ---------- TELEMETRY ----------
        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Shoot Index", shootIndex);
        telemetry.update();
    }
}
