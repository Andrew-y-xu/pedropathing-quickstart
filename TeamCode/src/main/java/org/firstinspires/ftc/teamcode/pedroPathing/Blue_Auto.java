package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue_Auto {
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.500, 122.000), new Pose(38.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(137.5),
                            Math.toRadians(137.5)
                    )
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(38.500, 104.000),
                                    new Pose(62.500, 85.000),
                                    new Pose(42.500, 83.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.500, 83.500), new Pose(19.000, 83.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.000, 83.500), new Pose(38.500, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(38.500, 104.000),
                                    new Pose(63.000, 88.000),
                                    new Pose(66.000, 62.000),
                                    new Pose(42.500, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.500, 60.000), new Pose(19.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.000, 60.000),
                                    new Pose(44.500, 70.000),
                                    new Pose(38.500, 104.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(38.500, 104.000),
                                    new Pose(65.000, 85.000),
                                    new Pose(65.000, 31.000),
                                    new Pose(42.500, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.500, 35.000), new Pose(19.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.000, 35.000),
                                    new Pose(70.000, 44.000),
                                    new Pose(38.500, 104.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5))
                    .build();
        }
    }
}
