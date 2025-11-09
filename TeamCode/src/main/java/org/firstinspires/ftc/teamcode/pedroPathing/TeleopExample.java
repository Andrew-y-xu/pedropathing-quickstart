package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleopExample extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    @Autonomous(name = "B   lue Auto", group = "Examples")
    public static class Blue_Auto extends OpMode {

        public static Follower follower;
        private Timer pathTimer, opmodeTimer;
        private int pathState;
        private Paths paths;

        public static void drawOnlyCurrent() {
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
        }

        public static void draw() {
            Drawing.drawDebug(follower);
        }

        /** ───────────────────────────────────────────────
         *  PATH DEFINITIONS (exported from PathBuilder)
         *  ─────────────────────────────────────────────── */
        public static class Paths {
            public PathChain Path1, Path2, Path3, Path4, Path5,
                    Path6, Path7, Path8, Path9, Path10;

            public Paths(Follower follower) {
                Path1 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(20.500, 122.000),
                                new Pose(38.500, 104.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(137.5),
                                Math.toRadians(137.5))
                        .build();

                Path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(38.500, 104.000),
                                new Pose(62.500, 85.000),
                                new Pose(42.500, 83.500)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(137.5),
                                Math.toRadians(180))
                        .build();

                Path3 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(42.500, 83.500),
                                new Pose(19.000, 83.500)))
                        .setTangentHeadingInterpolation()
                        .build();

                Path4 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(19.000, 83.500),
                                new Pose(38.500, 104.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(180),
                                Math.toRadians(137.5))
                        .build();

                Path5 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(38.500, 104.000),
                                new Pose(63.000, 88.000),
                                new Pose(66.000, 62.000),
                                new Pose(42.500, 60.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(137.5),
                                Math.toRadians(180))
                        .build();

                Path6 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(42.500, 60.000),
                                new Pose(19.000, 60.000)))
                        .setTangentHeadingInterpolation()
                        .build();

                Path7 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(19.000, 60.000),
                                new Pose(44.500, 70.000),
                                new Pose(38.500, 104.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(180),
                                Math.toRadians(137.5))
                        .build();

                Path8 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(38.500, 104.000),
                                new Pose(65.000, 85.000),
                                new Pose(65.000, 31.000),
                                new Pose(42.500, 35.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(137.5),
                                Math.toRadians(180))
                        .build();

                Path9 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(42.500, 35.000),
                                new Pose(19.000, 35.000)))
                        .setTangentHeadingInterpolation()
                        .build();

                Path10 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(19.000, 35.000),
                                new Pose(70.000, 44.000),
                                new Pose(38.500, 104.000)))
                        .setLinearHeadingInterpolation(
                                Math.toRadians(180),
                                Math.toRadians(137.5))
                        .build();
            }
        }

        /** ───────────────────────────────────────────────
         *  INIT PHASE
         *  ─────────────────────────────────────────────── */
        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            paths = new Paths(follower);

            follower.setStartingPose(new Pose(20.5, 122, Math.toRadians(137.5)));

            pathTimer = new Timer();
            opmodeTimer = new Timer();

            telemetry.addLine("Initialized Blue Auto!");
            telemetry.update();
            drawOnlyCurrent();
        }

        /** ───────────────────────────────────────────────
         *  START PHASE
         *  ─────────────────────────────────────────────── */
        @Override
        public void start() {
            opmodeTimer.resetTimer();
            pathTimer.resetTimer();
            pathState = 0; // start at first path
        }

        /** ───────────────────────────────────────────────
         *  MAIN LOOP — runs continuously during Auto
         *  ─────────────────────────────────────────────── */
        @Override
        public void loop() {
            follower.update();          // must always run for movement
            autonomousPathUpdate();     // handles path transitions

            // Optional telemetry for debugging
            telemetry.addData("State", pathState);
            telemetry.addData("Pose X", follower.getPose().getX());
            telemetry.addData("Pose Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)",
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
            draw();
        }

        /** ───────────────────────────────────────────────
         *  STATE MACHINE — runs through all 10 paths
         *  ─────────────────────────────────────────────── */
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.Path1);
                    setPathState(1);
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path2, true);
                        setPathState(2);
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path3, true);
                        setPathState(3);
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path4, true);
                        setPathState(4);
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path5, true);
                        setPathState(5);
                    }
                    break;

                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path6, true);
                        setPathState(6);
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path7, true);
                        setPathState(7);
                    }
                    break;

                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path8, true);
                        setPathState(8);
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path9, true);
                        setPathState(9);
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path10, true);
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!follower.isBusy()) {
                        setPathState(-1); // finished
                    }
                    break;
            }
        }

        /** ───────────────────────────────────────────────
         *  UTILITY — reset path state timer
         *  ─────────────────────────────────────────────── */
        private void setPathState(int newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }
    }
}
