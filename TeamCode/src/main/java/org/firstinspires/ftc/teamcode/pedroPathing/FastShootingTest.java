package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Disabled
public class FastShootingTest extends OpMode {
    Servo slot1,slot2,slot3;
    DcMotor flywheel;
    public void init(){
        slot1=hardwareMap.get(Servo.class,"lift1");
        slot2=hardwareMap.get(Servo.class,"lift2");
        slot3=hardwareMap.get(Servo.class,"lift3");
        flywheel=hardwareMap.get(DcMotor.class,"testemotor");
    }
    public void loop(){
        if(gamepad1.a){
            flywheel.setPower(1);
        }
        else if(gamepad1.y){
            flywheel.setPower(0);
        }
        if(gamepad1.b){
            slot1.setPosition(0.3);
            slot1.setPosition(0.5225);
            slot2.setPosition(0.55);
            slot2.setPosition(0.3);
            slot3.setPosition(0.55);
            slot3.setPosition(0.325);
            slot1.setPosition(0.05);
            slot2.setPosition(0.745);
            slot3.setPosition(0.325);
        }
    }
}