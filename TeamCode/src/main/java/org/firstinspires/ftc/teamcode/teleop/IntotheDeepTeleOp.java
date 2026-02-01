package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
@Disabled
@TeleOp
public class IntotheDeepTeleOp extends OpMode {

    enum Mode {
        CONTROL_OUTTAKE,
        CONTROL_INTAKE
    }
    private Mode currentMode = Mode.CONTROL_OUTTAKE;
    public boolean bIsRedAlliance = true;

    enum IntakeMode {
        CONTROL_GRAB_SAMPLE_START,

        CONTROL_RELEASE_SAMPLE_IN_HUMAN_ZONE,
        CONTROL_GRAB_SAMPLE_POSITION_ARM_TO_SAMPLE,
        CONTROL_GRAB_SAMPLE_POSITION_CHECK_SAMPLE,
        CONTROL_GRAB_SAMPLE_POSITION_GRAB_SAMPLE,
        CONTROL_GRAB_SAMPLE_FLIP_TO_BACK,
        CONTROL_GRAB_SAMPLE_FINAL_POSITION,
        CONTROL_GRAB_SAMPLE_DEFAULT,
        CONTROL_GRAB_SAMPLE_POSITION_ARM_TO_SAMPLE_BY_USER_1,
        CONTROL_GRAB_SAMPLE_POSITION_ARM_TO_SAMPLE_BY_USER_2,

        CONTROL_RELEASE_SAMPLE
    }
    enum IntakeDirection {
        INTAKE_DIRECTION_STOP,
        INTAKE_DIRECTION_GRAB,
        INTAKE_DIRECTION_RELEASE
    }

    private IntakeMode intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_DEFAULT;
    private IntakeDirection intakeDirection = IntakeDirection.INTAKE_DIRECTION_STOP;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer = 0;

    IntotheDeepTeleOpBoard board = new IntotheDeepTeleOpBoard();
    double speed = 1;
    private ElapsedTime waitTimer = new ElapsedTime();  // Time into current state

    public ElapsedTime wristDelayTimer = new ElapsedTime();
    public boolean wristDelayStarted = false;


    @Override
    public void init(){
        board.initDrive(hardwareMap);
        board.initOuttake(hardwareMap);
    }
    @Override
    public void init_loop() {
        boolean gamePad1RightBumperIsPressed = ifPressed(gamepad1.right_bumper);

        board.outtakeSlide_targetAngle = 0;
        board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;

        board.outtake_ArmPos = 0.065; //back
        board.outtake_ClawPos = board.outtake_clawPos_CLOSE; // Close
        board.leftwristpos = 0.061;//primed for specimen
        board.rightwristpos = 0.401;//primed for specimen
        board.intake_Pos = board.flipback;

        //board.outtake_WristPos = board.Outtake_wristPos_INIT;

        board.setOuttakeArmPosition(board.outtake_ArmPos);
        board.setOuttakeClawPosition(board.outtake_ClawPos);
        board.setOuttakeWristPosition(board.leftwristpos, board.rightwristpos);
        board.intakewheelservo(0.5, 0.5);
        board.setIntakePosition(board.intake_Pos);

        board.outtakeSlide(board.outtake_slide_p, board.outtake_slide_i, board.outtake_slide_d,board.outtake_slide_f, board.outtakeSlide_targetAngle);
        board.intakeSlide(board.extSlide_p, board.extSlide_i, board.extSlide_d, board.extSlide_f, board.extSlide_targetAngle);

        if ((gamePad1RightBumperIsPressed) && (bIsRedAlliance == true)){
            bIsRedAlliance = false;
        }else if ((gamePad1RightBumperIsPressed) && (bIsRedAlliance != true)){
            bIsRedAlliance = true;
        }
        telemetry.addData("IsRedAlliance: ", bIsRedAlliance);
        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        double driveTrain0,driveTrain1,driveTrain2,driveTrain3;

        boolean gamePad2leftBumperPressed = ifPressed(gamepad2.left_bumper);
        boolean gamePad2rightBumperPressed = ifPressed(gamepad2.right_bumper);
        boolean gamePad2xIsPressed = ifPressed(gamepad2.x);
        boolean gamePad2aIsPressed = ifPressed(gamepad2.a);
        boolean gamePad2bIsPressed = ifPressed(gamepad2.b);
        boolean gamePad2yIsPressed = ifPressed(gamepad2.y);
        boolean gamePad2dPadLeftIsPressed = ifPressed(gamepad2.dpad_left);
        boolean gamePad2dPadRightIsPressed = ifPressed(gamepad2.dpad_right);
        boolean gamePad2dPadUpIsPressed = ifPressed(gamepad2.dpad_up);
        boolean gamePad2dPadDownIsPressed = ifPressed(gamepad2.dpad_down);
        boolean gamePad2BackIsPressed = ifPressed(gamepad2.back);


        boolean gamePad1xIsPressed = ifPressed(gamepad1.back);
        boolean gamePad1yIsPressed = ifPressed(gamepad1.y);
        boolean gamePad1aIsPressed = ifPressed(gamepad1.a);
        boolean gamePad1bIsPressed = ifPressed(gamepad1.b);
        boolean gamePad1dPadUpIsPressed = ifPressed(gamepad1.dpad_up);
        boolean gamePad1RightBumperIsPressed = ifPressed(gamepad1.right_bumper);
        boolean gamePad1LeftBumperIsPressed = ifPressed(gamepad1.left_bumper);
        boolean gamePad1dPadLeftIsPressed = ifPressed(gamepad1.dpad_left);
        boolean gamePad1dPadRightIsPressed = ifPressed(gamepad1.dpad_right);

        /***************************/
        /****Drive Train Related ***/
        /***************************/
        double vx = speed*(-gamepad1.left_stick_y * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double vy = speed*(gamepad1.left_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));
        double o = speed*(gamepad1.right_stick_x * (1 + gamepad1.left_trigger) * (1 - gamepad1.right_trigger));//*//some value;

        if (gamePad1yIsPressed){
            speed = 0.7;
        }
        if (gamePad1bIsPressed){
            speed = 0.3;
        }
        if (gamePad1aIsPressed){
            speed = 1.5;
        }

        double leftFrontPower  = vx + vy + o;
        double rightFrontPower = vx - vy - o;
        double leftBackPower   = vx - vy + o;
        double rightBackPower  = vx + vy - o;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // calculate the speed for each wheel and drive
        board.frontLeft(leftFrontPower);
        board.frontRight(rightFrontPower);
        board.backLeft(leftBackPower);
        board.backRight(rightBackPower);

        telemetry.addData("frontLeft", leftFrontPower);
        telemetry.addData("frontRight", rightFrontPower);
        telemetry.addData("backLeft", leftBackPower);
        telemetry.addData("backRight", rightBackPower);
        telemetry.addData("GamePad 2 mode:", currentMode.toString());



        /***************************/
        /****Drive Arm Related ***/
        /***************************/

        switch (currentMode){
            case CONTROL_INTAKE:
                if (gamePad2xIsPressed){ //changes to outtake mode
                    currentMode = Mode.CONTROL_OUTTAKE;

                    board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                    board.leftwristpos = board.leftwristgrabspecimen;
                    board.rightwristpos = board.rightwristgrabspecimen;
                    board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                    board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                    try {
                        board.intake_Pos = board.flipback;
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                    board.initleft = board.stopstop;
                    board.initright = board.stopstop;
                } else if (gamePad2aIsPressed) { //change to intake mode
                    currentMode = Mode.CONTROL_INTAKE;
                    intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_DEFAULT; //maybe change this setting to contain something? its redundant

                    board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                    board.leftwristpos = board.leftwristgrabspecimen;
                    board.rightwristpos = board.rightwristgrabspecimen;
                    board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                    board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                    board.intake_Pos = board.flipback;
                    board.initleft = board.stopstop;
                    board.initright = board.stopstop;


                }else{
                    /* This is to release the sample to human player */
                    if (gamePad2dPadDownIsPressed){
                        intakeMode = IntakeMode.CONTROL_RELEASE_SAMPLE;
                        intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                    }
                    // Open Intake slides and start to get sample
                    if (gamePad2dPadUpIsPressed) {
                        intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_START;
                        intakeDirection = IntakeDirection.INTAKE_DIRECTION_GRAB;
                        //intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_POSITION_CHECK_SAMPLE;

                        //intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_START;
                        //intakeDirection = IntakeDirection.INTAKE_DIRECTION_GRAB;
                    }

                    // Handling of intake direction
                    if (gamePad2dPadRightIsPressed){
                        intakeDirection = IntakeDirection.INTAKE_DIRECTION_GRAB;
                    }
                    if (gamePad2dPadLeftIsPressed) {
                        intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                        //board.intakeMotorDirectionReject = true;
                    }
                    if (gamePad2yIsPressed) {//return to init position
                        //intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_DEFAULT;
                        board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                        board.leftwristpos = board.leftwristgrabspecimen;
                        board.rightwristpos = board.rightwristgrabspecimen;
                        board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                        board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                        board.intake_Pos = board.flipback;
                        board.initleft = board.stopstop;
                        board.initright = board.stopstop;
                    }
                    if (gamePad2bIsPressed) {
                        intakeMode = IntakeMode.CONTROL_RELEASE_SAMPLE_IN_HUMAN_ZONE;
                        intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                    }

                    switch (intakeDirection){
                        case INTAKE_DIRECTION_STOP:
                            board.initleft = board.stopstop;
                            board.initright = board.stopstop;
                            break;
                        case INTAKE_DIRECTION_GRAB:
                            board.initleft = board.leftgo;
                            board.initright = board.rightgo;
                            //board.intakeMotorPower = board.intakePOWER_VALUE;
                            //board.intakeMotorDirectionReject = false;
                            break;
                        case INTAKE_DIRECTION_RELEASE:
                            board.initleft = board.leftreverse;
                            board.initright = board.rightreverse;
                            //board.intakeMotorPower = board.intakePOWER_VALUE;
                            //board.intakeMotorDirectionReject = true;
                            break;
                    }


                    switch (intakeMode){
                        case CONTROL_GRAB_SAMPLE_DEFAULT:
                            board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                            board.leftwristpos = board.leftwristgrabspecimen;
                            board.rightwristpos = board.rightwristgrabspecimen;
                            board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                            board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                            board.intake_Pos = board.flipback;
                            board.initleft = board.stopstop;
                            board.initright = board.stopstop;
                            break;

                        case CONTROL_GRAB_SAMPLE_START:
                            board.extSlide_targetAngle = board.INTAKE_SLIDE_DEFAULT;//extend slides
                            board.intake_Pos = board.flipdown;
                            intakeDirection = IntakeDirection.INTAKE_DIRECTION_GRAB;
                            intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_POSITION_CHECK_SAMPLE;
                            break;

                        case CONTROL_RELEASE_SAMPLE_IN_HUMAN_ZONE:
                            board.extSlide_targetAngle = board.SLIDE_RELEASE_HUMAN;
                            board.intake_Pos = board.flipdown;
                            intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                            break;


                        case CONTROL_GRAB_SAMPLE_POSITION_CHECK_SAMPLE:
                            // Here checks the color sensor, if it is wrong, release the sample.
                            if ((bIsRedAlliance == true) && board.intake_DetectOneColor(IntotheDeepTeleOpBoard.SAMPLE_COLOR.BLUE)) {
                                board.intake_Pos = board.flipdown;
                                intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                            }
                            if ((bIsRedAlliance == false) && board.intake_DetectOneColor(IntotheDeepTeleOpBoard.SAMPLE_COLOR.RED)) {
                                board.intake_Pos = board.flipdown;
                                intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                            }
                            // if color sensor is correct, restore the intake slide
                            if (((bIsRedAlliance == true) && board.intake_DetectTwoColor(IntotheDeepTeleOpBoard.SAMPLE_COLOR.RED, IntotheDeepTeleOpBoard.SAMPLE_COLOR.YELLOW)) ||
                                    ((bIsRedAlliance == false) && board.intake_DetectTwoColor(IntotheDeepTeleOpBoard.SAMPLE_COLOR.BLUE, IntotheDeepTeleOpBoard.SAMPLE_COLOR.YELLOW))) {

                                // Fully retract intake and outtake slides
                                board.extSlide_targetAngle = board.INTAKE_SLIDE_RESTORE;
                                board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;

                                // Stop intake servos and motor
                                board.intake_Pos = board.flipback;
                                //board.intakeMotorPower = 0;
                                board.initleft = board.stopstop;
                                board.initright = board.stopstop;

                                intakeDirection = IntakeDirection.INTAKE_DIRECTION_STOP;

                                // Keep intakeMode unchanged or set to an idle state if needed

                            }
                            break;

                        case CONTROL_RELEASE_SAMPLE:
                            board.extSlide_targetAngle = board.INTAKE_SLIDE_DEFAULT;//extend slides
                            board.intake_Pos = board.flipdown;
                            intakeDirection = IntakeDirection.INTAKE_DIRECTION_RELEASE;
                            break;
                    }

                    // here starts the control on intake
                    // Fine adjust for the intake slide
                    if (gamepad2.right_stick_y != 0) {
                        // Each time it adjust slide 200, the boundary is checked inside outtakeSlide()
                        board.extSlide_targetAngle = board.extSlide_currentAngle - gamepad2.right_stick_y * 200;
                    }
                }
                break;











            case CONTROL_OUTTAKE:
                if (gamePad2xIsPressed){
                    currentMode = Mode.CONTROL_OUTTAKE;

                    board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                    board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                    board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                    board.intake_Pos = board.flipback;
                    board.initleft = board.stopstop;
                    board.initright = board.stopstop;
                    board.leftwristpos = board.leftwristgrabspecimen;
                    board.rightwristpos = board.rightwristgrabspecimen;
                    //board.intakeMotorPower = 0;
                } else if (gamePad2aIsPressed) {
                    currentMode = Mode.CONTROL_INTAKE;
                    intakeMode = IntakeMode.CONTROL_GRAB_SAMPLE_DEFAULT;

                    board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                    board.outtakeSlide_targetAngle = board.outtakeSlide_RESTORE;
                    board.extSlide_targetAngle = board.INTAKE_SLIDE_FOR_CLAW_TO_GRAB;
                    board.intake_Pos = board.flipback;
                    board.initleft = board.stopstop;
                    board.initright = board.stopstop;
                    board.leftwristpos = board.leftwristgrabspecimen;
                    board.rightwristpos = board.rightwristgrabspecimen;
                    //board.intakeMotorPower = 0;
                }else {
                    // Outtake grab specimen
                    if (gamePad2dPadUpIsPressed) {
                        //board.outtakeCurrentMode = IntotheDeepTeleOpBoard.OuttakeMode.OUTTAKE_HIGH_BASKET_MODE;
                        // First, we bring back the arm to back position
                        board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                        board.leftwristpos = board.leftwristgrabspecimen;
                        board.rightwristpos = board.rightwristgrabspecimen;

                        //board.outtakeSlide_targetAngle = board.outtakeSlide_HIGH_BASKET;
                    }

                    // Outtake hang specimen
                    if (gamePad2dPadRightIsPressed) {
                        //board.outtakeCurrentMode = IntotheDeepTeleOpBoard.OuttakeMode.OUTTAKE_MIDDLE_BASKET_MODE;
                        // First, we bring back the arm to back position
                        board.outtake_ArmPos = board.outtake_Armpos_specimenhang;


                        board.leftwristpos = board.leftwristhangspecimen;
                        board.rightwristpos = board.rightwristhangspecimen;
                        //board.outtakeSlide_targetAngle = board.outtakeSlide_MIDDLE_BASKET;
                    }

                    /***************************/
                    /****MOST IMPORTANT ***/
                    /***************************/
/*
                    if (gamePad2dPadRightIsPressed) {
                        board.outtakeCurrentMode = IntotheDeepTeleOpBoard.OuttakeMode.OUTTAKE_SPECIMEN_HIGH_BAR;
                        // First, we bring back the arm to front position
                        board.outtake_ArmPos = board.outtake_ArmPos_FRONT;
                        board.outtake_WristPos = board.Outtake_wristPos_TURN180;
                        board.outtakeSlide_targetAngle = board.outtakeSlide_HIGH_SPECIMEN;
                    }

 */
                    /***************************/
                    /****MOST IMPORTANT ***/
                    /***************************/
/*
                    if (gamePad2dPadLeftIsPressed) {
                        board.outtakeCurrentMode = IntotheDeepTeleOpBoard.OuttakeMode.OUTTAKE_SPECIMEN_MIDDLE_BAR;
                        // First, we bring back the arm to front position
                        board.outtake_ArmPos = board.outtake_ArmPos_FRONT;
                        board.outtake_WristPos = board.Outtake_wristPos_TURN180;
                        board.outtakeSlide_targetAngle = board.outtakeSlide_MIDDLE_SPECIMEN;
                    }
                    */
                    if (gamePad2yIsPressed) {
                        //board.outtakeCurrentMode = IntotheDeepTeleOpBoard.OuttakeMode.OUTTAKE_GRAB_SPECIMEN;
                        // First, we bring back the arm to back position
                        board.outtake_ArmPos = board.outtake_Armpos_specimengrab;
                        board.leftwristpos = board.leftwristgrabspecimen;
                        board.rightwristpos = board.rightwristgrabspecimen;

                        //board.outtakeSlide_targetAngle = board.outtakeSlide_GRAB_SPECIMEN;
                    }
                    /*
                    if (gamePad2BackIsPressed){
                        board.outtake_ArmPos = board.outtake_ArmPos_INTAKE;
                        board.outtake_WristPos = board.Outtake_wristPos_INIT;
                        board.outtakeSlide_targetAngle = board.outtakeSlide_GRAB_SAMPLE_FROM_FLOOR;
                    } */


                    // Fine adjust for the outtake slide
                    if (gamepad2.right_stick_y != 0) {
                        // Each time it adjust slide 200, the boundary is checked inside outtakeSlide()
                        board.outtakeSlide_targetAngle = board.outtake_leftSlide_currentAngle - gamepad2.right_stick_y * 200;
                    }
                    // Fine tune arm degree, each time go up or down 0.05
                    if (gamepad2.left_stick_y != 0) {
                        board.outtake_ArmPos = board.outtake_ArmPos + 0.015 * gamepad2.left_stick_y;
                    }
                }
                break;
        }

        /***************************/
        /****Outtake Claw Related ***/
        /***************************/
        // Toggle the claw
        if (gamePad2rightBumperPressed && (board.outtake_ClawPos == board.outtake_clawPos_CLOSE)) {
            board.outtake_ClawPos = board.outtake_clawPos_OPEN;
        } else if (gamePad2rightBumperPressed && (board.outtake_ClawPos != board.outtake_clawPos_CLOSE)) {
            board.outtake_ClawPos = board.outtake_clawPos_CLOSE;
        }
        // Toggle the wrist
        /*
        if (gamePad2leftBumperPressed && (board.outtake_WristPos == board.Outtake_wristPos_TURN90)) {
            board.outtake_WristPos = board.Outtake_wristPos_INIT;
        } else if (gamePad2leftBumperPressed && (board.outtake_WristPos != board.Outtake_wristPos_TURN90)) {
            board.outtake_WristPos = board.Outtake_wristPos_TURN90;
        }
        */
        //Reset after each loop
        booleanIncrementer = 0;

        /***************************/
        /**** Calling TeleopBoard ***/
        /***************************/

        board.setOuttakeArmPosition(board.outtake_ArmPos);
        board.setOuttakeClawPosition(board.outtake_ClawPos);
        board.setOuttakeWristPosition(board.leftwristpos, board.rightwristpos);


        board.setIntakePosition(board.intake_Pos);
        board.intakewheelservo(board.initleft, board.initright);
        //board.setIntakeMotorPower(board.intakeMotorPower,board.intakeMotorDirectionReject);


        board.outtakeSlide(board.outtake_slide_p, board.outtake_slide_i, board.outtake_slide_d,board.outtake_slide_f, board.outtakeSlide_targetAngle);
        board.intakeSlide(board.extSlide_p, board.extSlide_i, board.extSlide_d, board.extSlide_f, board.extSlide_targetAngle);

        // Toggle select red or blue alliance.
        if ((gamePad1RightBumperIsPressed) && (bIsRedAlliance == true)){
            bIsRedAlliance = false;
        }else if ((gamePad1RightBumperIsPressed) && (bIsRedAlliance != true)){
            bIsRedAlliance = true;
        }
        telemetry.addData("Intake mode:", intakeMode.toString());

        telemetry.addData("IsRedAlliance: ", bIsRedAlliance);
        telemetry.addData("intake slide target: ", board.extSlide_targetAngle);
        telemetry.addData("intake slide current pos: ", board.extSlide_currentAngle);

        telemetry.update();
    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }
        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);
        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }
}
