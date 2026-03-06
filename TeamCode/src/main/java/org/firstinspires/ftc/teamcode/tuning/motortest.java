package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AutoShooter;
import org.firstinspires.ftc.teamcode.common.AutoFlicker;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name="motor tuning")
public class motortest extends OpMode {
    DcMotor testthemotor;

    @Override
    public void init() {
        testthemotor = hardwareMap.dcMotor.get("testthemotor");
    }
    @Override
    public void loop() {
        if(){

        }
    }

}
