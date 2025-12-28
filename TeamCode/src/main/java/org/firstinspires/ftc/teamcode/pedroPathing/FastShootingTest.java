package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
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
            flywheel.setPower(1-flywheel.getPower());
        }
        if(gamepad1.b){
            slot1.setPosition(0.685);
            slot1.setPosition(0.505);
            slot2.setPosition(0.685);
            slot2.setPosition(0.505);
            slot3.setPosition(0.685);
            slot3.setPosition(0.505);
            slot1.setPosition(0.325);
            slot2.setPosition(0.325);
            slot3.setPosition(0.325);
        }
    }
}
