package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
@Disabled
public class IntotheDeepTeleOpBoard {
    /***************************/
    /****Drive Train Related ***/
    /***************************/
    //This is to put common code into one class
    //THIS DOES NOT extends OpMode!!!!
    //We define this ourselves
    //This is something that you can can call to get values and stuff for other programs
    //DigitalChannel is like a binary sensor/button(for example, the A button on the gamepad)
    //DcMotor is a type of motor
    private DcMotorEx motor_frontLeft;
    private DcMotorEx motor_frontRight;
    private DcMotorEx motor_backLeft;
    private DcMotorEx motor_backRight;

    /***************************/
    /****Outtake Related ***/
    /***************************/
    enum OuttakeMode {
        OUTTAKE_HIGH_BASKET_MODE,
        OUTTAKE_MIDDLE_BASKET_MODE,
        OUTTAKE_SPECIMEN_HIGH_BAR,
        OUTTAKE_SPECIMEN_MIDDLE_BAR,
        OUTTAKE_GRAB_SPECIMEN,
        OUTTAKE_RESTORE
    }

    private PIDController outtake_leftSlide_controller;
    private PIDController outtake_rightSlide_controller;
    public static double outtake_slide_p = 0.01, outtake_slide_i = 0, outtake_slide_d = 0.0002;
    public static double outtake_slide_f = 0.18; // this is for feedforward
    private double outtake_slide_init_angle = 0;
    private double OUTTAKE_SLIDE_MAX = 4330;//4330 originally
    double outtakeSlide_HIGH_BASKET = 4330;//4330 originally
    double outtakeSlide_MIDDLE_BASKET = 2500;//2500 originally
    double outtakeSlide_HIGH_SPECIMEN = 2000;//2000 originally
    double outtakeSlide_MIDDLE_SPECIMEN = 1000;//1000 originally
    double outtakeSlide_GRAB_SPECIMEN = 0;

    double outtakeSlide_GRAB_INTAKE_CAN_SWING_ARM = 1200;//1200 originally
    double outtakeSlide_GRAB_INTAKE_CAN_SWING_ARM_HIGH = 2500; //2500 originally
    double outtakeSlide_GRAB_INTAKE = 600; //600 originally
    double outtakeSlide_GRAB_SAMPLE_FROM_FLOOR = 100; //100 originally
    double outtakeSlide_RESTORE = 0; //0 originally

    double outtake_ArmPos = 0.065; //Back //0 originally
    double outtake_ClawPos = 1; // Close //1 originally
    double outtake_ArmPos_MAX = 0.85;//0.88; //0.85 originally
    double outtake_ArmPos_BACK = 0.03; //0.03 originally
    double outtake_ArmPos_BACKToDrop = 0.05; //0.05 originally
    double outtake_ArmPos_FRONT = 0.48; //0.48 originally
    double outtake_ArmPos_INTAKE = 0.83;//0.88; //0.83 originally



    //new stuff for arm
    double outtake_Armpos_specimengrab = 0.065;
    double outtake_Armpos_specimenhang = 0.605;
    double outtake_Armpos_afterhang = 0.715;
    //new stuff for wrist
    double leftwristpos = 0.061;
    double rightwristpos = 0.401;
    double leftwristgrabspecimen = 0.061;
    double rightwristgrabspecimen = 0.401;
    double leftwristhangspecimen = 0.844;
    double rightwristhangspecimen = 0.714;
    double outtake_leftWristPos = 0.5;
    double outtake_rightWristPos = 0.5;





    double outtake_clawPos_OPEN = 0.35; //0.35 originally
    double outtake_clawPos_FULLOPEN = 0;//0 originally
    double outtake_clawPos_CLOSE = 1.0;// originally 0.94
    public static double outtakeSlide_targetAngle = 0; // originally 0
    public double outtake_leftSlide_currentAngle;
    public double outtake_rightSlide_currentAngle;





    double outtake_ArmPos_INIT = 0; // originally 0
    double outtake_clawPos_INIT = 0; // Open = 0; Close = 1 // originally0
    double outtake_ArmPos_TOTAL_DEGREE = 270; // originally 270
    //double linearRail_LEFT_TOTAL_DEGREE = 82;
    //double linearRailLeftPos = 1; // Close
    //double linearRailRightPos = 1; // Close
    //double linearRailLeftPos_CLOSE = 1;
    //double linearRailLeftPos_OPEN = 0;
    //double lineaRailRightPos_CLOSE = 1;
    //double lineaRailRightPos_IntakePos = 0.3;
    //double lineaRailRightPos_OPEN = 0;
    //double linearRailLeftPos_INIT = 0.9;
    //double linearRailRightPos_INIT = 1;


    // Outside gear reduction 86:28 = 43:14, bare motor ticks per rotation = 28
    // RevRoboticsUltraplanetaryHDHexMotor gear rem,duction = 20:1 (5:1 and 4:1)(actual ratio is 68/13 for each of 5:1, and 76/21 for 4:1)
    //private final double slide_exactRatio = 68*76/(13*21);
    //private final double slide_ticks_in_degree = 28*slide_exactRatio*43/(14*360.0);//2.4

    // For Gobilda slide:
    // bare motor ticks per rotation = 28
    // gear reduction = 19.2:1 for Gobilda 312, 13.7:1 for Gobilda 435
    // Outside gear reduction = 1:1
    private final double outtake_slide_exactRatio = 13.7;//(((1+(46/17))) * (1+(46/11)))
    private final double outtake_slide_ticks_in_degree = 28*outtake_slide_exactRatio/360.0;
    private DcMotorEx outtake_liftLeft;
    private DcMotorEx outtake_liftRight;
    double outtake_slide_pidMax = 0; //// originally 0

    // For Gobilda slide:
    // bare motor ticks per rotation = 28
    // gear reduction = 19.2:1 for Gobilda 312, 13.7:1 for Gobilda 435, 5.2:1 for Gobilda 1150
    // Outside gear reduction = 1:1
    private final double extSlide_exactRatio = 5.2;//(((1+(46/17))) * (1+(46/11)))
    private final double extSlide_ticks_in_degree = 28*extSlide_exactRatio/360.0;
    private DcMotorEx extSlide;
    private PIDController extSlide_controller;
    public static double extSlide_p = 0.01, extSlide_i = 0, extSlide_d = 0.0002;
    public static double extSlide_f = 0;
    double extSlide_pidMax = 0;
    public double extSlide_currentAngle;
    public static double extSlide_targetAngle = 0;
    private double extSlide_init_angle = 0;
    private double INTAKE_SLIDE_MAX = 1450; // originally 1450
    public double INTAKE_SLIDE_DEFAULT = 1400; // originally 1400
    public double SLIDE_RELEASE_HUMAN = 700; // originally 1400
    public double INTAKE_SLIDE_RESTORE = 0;
    public double INTAKE_SLIDE_FOR_CLAW_TO_GRAB = 0;


    public OuttakeMode outtakeCurrentMode = OuttakeMode.OUTTAKE_RESTORE;

    public Servo outtake_armLeft;
    public Servo outtake_armRight;
    private Servo leftwrist;
    private Servo rightwrist;

    /*
    public Servo linearRailLeft;
    public Servo linearRailRight;
    */
    AnalogInput outtake_armLeft_analogInput;
    double get_outtake_armLeftPos = 0;
    /*
    AnalogInput linearRailLeft_analogInput;
    AnalogInput linearRailRight_analogInput;
    double get_linearRailLeftPos = 0;
    double get_linearRailRightPos = 0;
    */

    enum SAMPLE_COLOR {
        YELLOW,
        RED,
        BLUE
    }
    public Servo outtake_claw;
    DigitalChannel brushlandColorPin0;
    DigitalChannel brushlandColorPin1;

    public Servo outtake_wrist;
    public double Outtake_wristPos_INIT = 0.5; // This is to horizontal to the arm // originally 0
    public double Outtake_wristPos_TURN90 = 0.25; // This is to turn 90 degree // originally 0.29
    public double Outtake_wristPos_TURN180 = 0.75; // This is to turn 180 degree // originally 0.56

    public double outtake_WristPos = Outtake_wristPos_INIT; // Close

    public Servo intake;
    public Servo leftwheel;
    public Servo rightwheel;

    public double INTAKE_POS_STOP = 0.15; // originally 0.15
    public double INTAKE_POS_FACE_FLOOR = 0.4; // originally 0.4
//    public double INTAKE_POS_RELEASE = 1;
    public double intake_Pos = INTAKE_POS_STOP;//0.5 is stop, 0 is grab, 1 is release

    public DcMotorEx intakeMotor;
    public double intakeMotorPower = 0;
    public boolean intakeMotorDirectionReject = false;
    public double intakePOWER_VALUE = 0.8; // originally 0.8

    public double initleft = 0.5;
    public double initright = 0.5;
    public double leftgo = 1.0;
    public double rightgo = 0.0;
    public double leftreverse = 0.0;
    public double rightreverse = 1.0;
    public double stopstop = 0.5;
    public double flipback = 0.88;
    public double flipdown = 0.0;



    public void initDrive(HardwareMap hwMap){

        //initialize a motor in the code
        motor_frontLeft = hwMap.get(DcMotorEx.class, "left_front_drive");
        motor_frontRight = hwMap.get(DcMotorEx.class, "right_front_drive");
        motor_backLeft = hwMap.get(DcMotorEx.class, "left_back_drive");
        motor_backRight = hwMap.get(DcMotorEx.class, "right_back_drive");

        motor_frontLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_backRight.setDirection(DcMotor.Direction.FORWARD);
        motor_frontRight.setDirection(DcMotor.Direction.FORWARD);
        motor_backLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //motor run mode
        motor_frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor_frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }
    /*
    public void drivestop(){
        motor_frontLeft.setPower(0);
        motor_frontRight.setPower(0);
        motor_backLeft.setPower(0);
        motor_backRight.setPower(0);
    }
    */
    public void frontLeft(double power){
        if (power > 1){
            power = 1;
        }
        motor_frontLeft.setPower(power);
    }
    public void frontRight(double power){
        if (power > 1){
            power = 1;
        }
        motor_frontRight.setPower(power);
    }
    public void backLeft(double power){
        if (power > 1){
            power = 1;
        }
        motor_backLeft.setPower(power);
    }
    public void backRight(double power){
        if (power > 1){
            power = 1;
        }
        motor_backRight.setPower(power);
    }

    public void initOuttake(HardwareMap hardwareMap){
        outtake_leftSlide_controller = new PIDController(outtake_slide_p, outtake_slide_i, outtake_slide_d);
        outtake_rightSlide_controller = new PIDController(outtake_slide_p, outtake_slide_i, outtake_slide_d);
        extSlide_controller = new PIDController(extSlide_p,extSlide_i, extSlide_d);

        outtake_liftLeft = hardwareMap.get(DcMotorEx.class, "leftSlide");
        outtake_liftLeft.setDirection(DcMotorEx.Direction.FORWARD); //CAYDEN CHANGE BEFORE REVERSE
        outtake_liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake_liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake_liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        outtake_liftRight = hardwareMap.get(DcMotorEx.class, "rightSlide");
        outtake_liftRight.setDirection(DcMotorEx.Direction.REVERSE);  //CAYDEN CHANGE BEFORE FORWARD
        outtake_liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake_liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake_liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extSlide = hardwareMap.get(DcMotorEx.class, "extSlide");
        extSlide.setDirection(DcMotorEx.Direction.FORWARD);
        extSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        outtake_armLeft = hardwareMap.get(Servo.class, "armLeft");
        outtake_armRight = hardwareMap.get(Servo.class, "armRight");
        leftwrist = hardwareMap.get(Servo.class, "wristLeft");
        rightwrist = hardwareMap.get(Servo.class, "wristRight");

        /*
        linearRailLeft = hardwareMap.get(Servo.class, "linearRailLeft");
        linearRailRight = hardwareMap.get(Servo.class, "linearRailRight");
        */
        //outtake_armLeft_analogInput = hardwareMap.get(AnalogInput.class, "armLeft_analoginput");//not used
        /*
        linearRailLeft_analogInput = hardwareMap.get(AnalogInput.class, "linearRailLeft_analoginput");
        linearRailRight_analogInput = hardwareMap.get(AnalogInput.class, "linearRailRight_analoginput");
        */

        outtake_claw = hardwareMap.get(Servo.class, "claw");
        //claw_analogInput = hardwareMap.get(AnalogInput.class, "claw_analoginput");

        intake = hardwareMap.get(Servo.class, "flip");

        leftwheel = hardwareMap.get(Servo.class, "left");
        rightwheel = hardwareMap.get(Servo.class, "right");
        leftwheel.getController().pwmEnable();
        rightwheel.getController().pwmEnable();

        brushlandColorPin0 = hardwareMap.get(DigitalChannel.class, "brushlandColorIntakePin0");
        brushlandColorPin0.setMode(DigitalChannel.Mode.INPUT);// by default
        brushlandColorPin1 = hardwareMap.get(DigitalChannel.class, "brushlandColorIntakePin1");
        brushlandColorPin1.setMode(DigitalChannel.Mode.INPUT);// by default


        //intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
       // intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);




        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    /*
    public void setIntakeMotorPower(double power, boolean intakeMotorDirectionReject){
        if (power > 1){
            power = 1;
        }
        if (intakeMotorDirectionReject == true){
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        }
        else{
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        intakeMotor.setPower(power);

    }
    */
    /*
    public double getOuttakeLinearRailLeftPosition(){
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        get_linearRailLeftPos = linearRailLeft_analogInput.getVoltage()/ 3.3 * 360;
        return get_linearRailLeftPos;
    }
    public double getOuttakeLinearRailRightPosition(){
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        get_linearRailRightPos = linearRailRight_analogInput.getVoltage()/ 3.3 * 360;
        return get_linearRailRightPos;
    }
    */
    /*
    public double getOuttakeArmLeftPosition(){
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        get_outtake_armLeftPos = outtake_armLeft_analogInput.getVoltage()/ 3.3 * 360;
        return get_outtake_armLeftPos;
    }
    */
    public void setOuttakeWristPosition(double leftwristposition, double rightwristposition) {

        leftwrist.setPosition(leftwristposition);
        rightwrist.setPosition(rightwristposition);
    }
    public void setOuttakeClawPosition(double position) {
        if (position > 1){
            position = 1;
        }
        if (position < 0) {position = 0;}

        outtake_claw.setPosition(position);
    }
    /*
    public void setOuttakeClaw_OPEN() {
        outtake_claw.setPosition(outtake_clawPos_OPEN);
    }
    public void setOuttakeClaw_CLOSE() {
        outtake_claw.setPosition(outtake_clawPos_CLOSE);
    }
    */

//    public void closeOuttakeClaw_DetectOneColor(SAMPLE_COLOR sample_color) {
//        boolean pin0 =  brushlandColorPin0.getState();
//        boolean pin1 =  brushlandColorPin1.getState();
//
//        if ((sample_color == SAMPLE_COLOR.YELLOW) && (pin0 == true) && (pin1 == true)) // Yellow
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//        else if ((sample_color == SAMPLE_COLOR.RED) && (pin0 == true) && (pin1 == false)) // RED
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//        else if ((sample_color == SAMPLE_COLOR.BLUE) && (pin0 == false) && (pin1 == true)) // BLUE
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//    }
//
//    public void closeOuttakeClaw_DetectTwoColor(SAMPLE_COLOR sample_color1, SAMPLE_COLOR sample_color2) {
//        boolean pin0 =  brushlandColorPin0.getState();
//        boolean pin1 =  brushlandColorPin1.getState();
//
//        if (((sample_color1 == SAMPLE_COLOR.YELLOW) || (sample_color2 == SAMPLE_COLOR.YELLOW)) && (pin0 == true) && (pin1 == true)) // Yellow
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//        if (((sample_color1 == SAMPLE_COLOR.RED) || (sample_color2 == SAMPLE_COLOR.RED)) && (pin0 == true) && (pin1 == false)) // RED
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//        else if (((sample_color1 == SAMPLE_COLOR.BLUE) || (sample_color2 == SAMPLE_COLOR.BLUE)) && (pin0 == false) && (pin1 == true)) // BLUE
//        {
//            outtake_claw.setPosition(outtake_clawPos_CLOSE);
//        }
//    }

    public boolean intake_DetectOneColor(SAMPLE_COLOR sample_color) {
        boolean pin0 =  brushlandColorPin0.getState();
        boolean pin1 =  brushlandColorPin1.getState();

        if ((sample_color == SAMPLE_COLOR.YELLOW) && (pin0 == true) && (pin1 == true)) // Yellow
        {
            return true;
        }
        else if ((sample_color == SAMPLE_COLOR.RED) && (pin0 == true) && (pin1 == false)) // RED
        {
            return true;
        }
        else if ((sample_color == SAMPLE_COLOR.BLUE) && (pin0 == false) && (pin1 == true)) // BLUE
        {
            return true;
        }
        return false;
    }
    public boolean intake_DetectTwoColor(SAMPLE_COLOR sample_color1, SAMPLE_COLOR sample_color2) {
        boolean pin0 =  brushlandColorPin0.getState();
        boolean pin1 =  brushlandColorPin1.getState();

        if (((sample_color1 == SAMPLE_COLOR.YELLOW) || (sample_color2 == SAMPLE_COLOR.YELLOW)) && (pin0 == true) && (pin1 == true)) // Yellow
        {
            return true;
        }
        if (((sample_color1 == SAMPLE_COLOR.RED) || (sample_color2 == SAMPLE_COLOR.RED)) && (pin0 == true) && (pin1 == false)) // RED
        {
            return true;
        }
        if (((sample_color1 == SAMPLE_COLOR.BLUE) || (sample_color2 == SAMPLE_COLOR.BLUE)) && (pin0 == false) && (pin1 == true)) // BLUE
        {
            return true;
        }
        return false;
    }
    public void setOuttakeArmPosition(double position) {
        if (position > outtake_ArmPos_MAX){
            position = outtake_ArmPos_MAX;
        }
        if (position < 0) {position = 0;}
        outtake_armLeft.setPosition(position);
        outtake_armRight.setPosition(position);
    }
    /*
    public void setLinearRailLeftPosition(double position) {
        if (position > 1){
            position = 1;
        }
        if (position < 0) {position = 0;}

        linearRailLeft.setPosition(position);
    }
    public void setLinearRailRightPosition(double position) {
        if (position > 1){
            position = 1;
        }
        if (position < 0) {position = 0;}

        linearRailRight.setPosition(position);
    }
    */


    public void outtakeSlide(double p, double i, double d, double f, double targetAngle){
        outtake_leftSlide_controller.setPID(p, i, d);
        outtake_rightSlide_controller.setPID(p, i, d);
        outtake_leftliftSlide_currentPosition();
        outtake_rightliftSlide_currentPosition();

        // Boundary Check
        if (targetAngle > OUTTAKE_SLIDE_MAX){
            targetAngle = OUTTAKE_SLIDE_MAX;
        }
        if (targetAngle < 0) {
            targetAngle = 0;
        }

        double outtake_leftslide_pid = outtake_leftSlide_controller.calculate(outtake_leftSlide_currentAngle,targetAngle);
        double outtake_rightslide_pid = outtake_rightSlide_controller.calculate(outtake_rightSlide_currentAngle,targetAngle);

        double outtake_slide_ff = f;

        double outtake_leftslide_power = outtake_leftslide_pid + outtake_slide_ff;
        double outtake_rightSlide_power = outtake_rightslide_pid + outtake_slide_ff;

        if (outtake_leftslide_power > outtake_slide_pidMax) { // This is just to record the max power we use
            outtake_slide_pidMax = outtake_leftslide_power;
        }
        if (outtake_leftslide_power > 1.0) {
            outtake_leftslide_power = 1.0;
        }
        if (outtake_rightSlide_power > 1.0) {
            outtake_rightSlide_power = 1.0;
        }
        if (outtake_leftslide_power < -1.0) {
            outtake_leftslide_power = -1.0;
        }
        if (outtake_rightSlide_power < -1.0) {
            outtake_rightSlide_power = -1.0;
        }
        outtake_liftLeft.setPower(outtake_leftslide_power);
        outtake_liftRight.setPower(outtake_rightSlide_power);
    }


    public void outtake_leftliftSlide_currentPosition(){
        double liftSlidePosLeft = outtake_liftLeft.getCurrentPosition();
        //double liftSlidePosRight = liftRight.getCurrentPosition();
        //double liftSlidePosRight = liftSlidePosLeft;

        outtake_leftSlide_currentAngle = liftSlidePosLeft/outtake_slide_ticks_in_degree+outtake_slide_init_angle;
    }
    public void outtake_rightliftSlide_currentPosition(){
        double liftSlidePosRight = outtake_liftRight.getCurrentPosition();
        //double liftSlidePosRight = liftSlidePosLeft;

        outtake_rightSlide_currentAngle = liftSlidePosRight/outtake_slide_ticks_in_degree+outtake_slide_init_angle;
    }

    public void intakeSlide(double p, double i, double d, double f, double targetAngle){
        extSlide_controller.setPID(p, i, d);
        extSlide_currentPosition();

        // Boundary Check
        if (targetAngle > INTAKE_SLIDE_MAX){
            targetAngle = INTAKE_SLIDE_MAX;
        }

        double intake_slide_pid = extSlide_controller.calculate(extSlide_currentAngle,targetAngle);

        double intake_slide_ff = f;

        double intake_slide_power = intake_slide_pid + intake_slide_ff;

        if (intake_slide_power > 1.0) {
            intake_slide_power = 1.0;
        }
        if (intake_slide_power < -1.0) {
            intake_slide_power = -1.0;
        }
        extSlide.setPower(intake_slide_power);
    }
    public void extSlide_currentPosition(){
        double extSlidePos = extSlide.getCurrentPosition();
        //double liftSlidePosRight = liftSlidePosLeft;

        extSlide_currentAngle = extSlidePos/extSlide_ticks_in_degree+extSlide_init_angle;
    }
    public void setIntakePosition(double position) {
        intake.setPosition(position);
    }
    public void intakewheelservo(double dingus1, double dingus2) {
        leftwheel.setPosition(dingus1);
        rightwheel.setPosition(dingus2);
    }

    /*
    public void setIntake_GrabSample() {
        intake.setPosition(0);
    }
    public void setIntake_ReleaseSample() {
        intake.setPosition(1);
    }
    public void setIntake_Stop() {
        intake.setPosition(0.15);
    }
    */

}
