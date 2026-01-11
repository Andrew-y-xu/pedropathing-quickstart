import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Button Control", group = "TeleOp")
public class MotorButtonControl1 extends LinearOpMode {

    DcMotor motor;

    // Set your desired speeds here
    static final double LEFT_SPEED  = -0.06;  // X butto//.n
    static final double RIGHT_SPEED =  0.06;  // B button

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "left_drive");

        telemetry.addData(">", "Press Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                // Move left
                motor.setPower(LEFT_SPEED);
            }
            else if (gamepad1.b) {
                // Move right
                motor.setPower(RIGHT_SPEED);
            }
            else {
                // Stop motor
                motor.setPower(0);
            }

            telemetry.addData("X", gamepad1.x);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
