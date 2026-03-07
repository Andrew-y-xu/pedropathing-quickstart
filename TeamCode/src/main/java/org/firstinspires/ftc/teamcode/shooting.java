package org.firstinspires.ftc.teamcode;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// For Limelight
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
@Disabled
@TeleOp(name="Tuning AutoShoot", group="Test")
public class shooting extends OpMode {
    DcMotorEx testmotor;
    Servo hoodservo;

    double shooter_value = 0.0;
    double servo_value = 0.5;

    boolean xAlrPressed = false;
    boolean yAlrPressed = false;

    ElapsedTime motorDebounce = new ElapsedTime();
    ElapsedTime servoDebounce = new ElapsedTime();

    Limelight3A limelight;
    double limelight_ty;

    /**********
     * Init
     **********/
    @Override
    public void init() {
        //***** FlyWheel Motor
        testmotor = hardwareMap.get(DcMotorEx.class, "testemotor");
        testmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //testmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //testmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        testmotor.setDirection(DcMotorEx.Direction.REVERSE);

        //***** ShooterActuator Servo
        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        //***** Limelight
        limelight = hardwareMap.get(Limelight3A.class, "lookylookyseesee");

        //***** Initial Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**********
     * Loop
     **********/
    @Override
    public void loop() {

        //***** MOTOR CONTROL (X / Y)
        if (motorDebounce.milliseconds() > 300) {
            if (gamepad1.x && !xAlrPressed) {
                shooter_value += 0.1;
                motorDebounce.reset();
            }

            if (gamepad1.y && !yAlrPressed) {
                shooter_value -= 0.1;
                motorDebounce.reset();
            }
        }
        xAlrPressed = gamepad1.x;
        yAlrPressed = gamepad1.y;

        if (gamepad1.right_bumper) {
            shooter_value = 0.0;
            servo_value = 0.5;
        }
        testmotor.setPower(shooter_value);

        //***** SERVO CONTROL (A / B)
        if (servoDebounce.milliseconds() > 150) {
            if (gamepad1.a) {
                servo_value += 0.05;
                servoDebounce.reset();
            } else if (gamepad1.b) {
                servo_value -= 0.05;
                servoDebounce.reset();
            }
        }

        //***** Limelight feedback
        LLResult resultsOfLimelight = limelight.getLatestResult();

        if (resultsOfLimelight != null && resultsOfLimelight.isValid()) {
            limelight_ty = resultsOfLimelight.getTy();
        } else {
            limelight_ty = 0;
        }

        //***** Telemetry feedback
        servo_value = Math.max(0.0, Math.min(servo_value, 1.0));
        hoodservo.setPosition(servo_value);

        telemetry.addData("Motor Power", shooter_value);
        telemetry.addData("Hood Position", servo_value);
        telemetry.addData("LimeLight(ty)", limelight_ty);

        telemetry.update();
    }
}