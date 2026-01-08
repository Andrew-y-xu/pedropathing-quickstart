package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestAhh extends OpMode {
    Servo[] slot;
    private int servostate;
    public void init(){
        slot = new Servo[3];
        slot[0]=hardwareMap.get(Servo.class,"lift1");
        slot[1]=hardwareMap.get(Servo.class,"lift2");
        slot[2]=hardwareMap.get(Servo.class,"lift3");
    }
    public void loop(){
        if(gamepad1.dpad_up){
            slot[0].setPosition(0.3);
            slot[1].setPosition(0.55);
            slot[2].setPosition(0.55);
        }
        if(gamepad1.dpad_left||gamepad1.dpad_right) {
            slot[0].setPosition(0.5225);
            slot[1].setPosition(0.3);
            slot[2].setPosition(0.325);
        }
        if(gamepad1.dpad_down){
            slot[0].setPosition(0.745);
            slot[1].setPosition(0.05);
            slot[2].setPosition(0.1);
        }
    }
}
