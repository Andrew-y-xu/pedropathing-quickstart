package org.firstinspires.ftc.teamcode.pedroPathing.tuningPath;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TestPedroPath1 {

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
    public PathChain Path11;

    public TestPedroPath1(Follower follower) {
        this.Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(122.565, 124.031),
                        new Pose(88.672, 93.252)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(38))
                .setReversed()
                .build();

        this.Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(88.672, 93.252),
                        new Pose(85.924, 78.229),
                        new Pose(129.344, 84.092)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                .build();

        this.Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(129.344, 84.092),
                        new Pose(89.221, 83.359)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                .build();

        this.Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(89.221, 83.359),
                        new Pose(81.527, 72.000),
                        new Pose(81.710, 55.511),
                        new Pose(136.122, 58.260)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                .build();

        this.Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(136.122, 58.260),
                        new Pose(105.344, 61.008),
                        new Pose(128.244, 71.450)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        this.Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(128.244, 71.450),
                        new Pose(80.244, 75.115)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(38))
                .build();

        this.Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(80.244, 75.115),
                        new Pose(62.107, 47.267),
                        new Pose(93.252, 28.580),
                        new Pose(134.107, 34.809)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                .build();

        this.Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(134.107, 34.809),
                        new Pose(75.664, 23.817)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))
                .build();

        this.Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(75.664, 23.817),
                        new Pose(134.656, 9.160)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))
                .build();

        this.Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(134.656, 9.160),
                        new Pose(76.031, 23.634)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(54))
                .build();

        this.Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(76.031, 23.634),
                        new Pose(107.908, 71.817)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(90))
                .build();
    }
}

