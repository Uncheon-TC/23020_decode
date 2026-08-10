package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
public class BlueFar {

    public static class Paths {

        public PathChain MainChain;

        public Paths(Follower follower) {

            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.824, 11.174),
                                    new Pose(13.869, 9.304)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(
                            new BezierCurve(
                                    new Pose(13.869, 9.304),
                                    new Pose(7.390, 35.647),
                                    new Pose(11.085, 53.687)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(11.085, 53.687),
                                    new Pose(59.607, 12.260)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(78),
                            Math.toRadians(180)
                    )

                    .build();
        }
    }
}
