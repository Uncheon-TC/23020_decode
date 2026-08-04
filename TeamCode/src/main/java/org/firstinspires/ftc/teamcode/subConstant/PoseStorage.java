package org.firstinspires.ftc.teamcode.subConstant;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class PoseStorage {
    private static Pose autoEndPose;
    private static ShooterConst.Goal autoGoal;

    private PoseStorage() {
    }

    // Auto에서 현재 Pose를 주기적으로 저장하면 마지막 값이 TeleOp으로 전달된다.
    public static void saveAutoPose(Pose pose, ShooterConst.Goal goal) {
        if (pose != null) {
            autoEndPose = pose;
            autoGoal = goal;
        }
    }

    public static void saveAutoPose(Pose2D pose, ShooterConst.Goal goal) {
        if (pose != null) {
            saveAutoPose(new Pose(
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.RADIANS)), goal);
        }
    }

    public static void saveAutoPose(double x, double y, double headingDegrees,
                                    ShooterConst.Goal goal) {
        saveAutoPose(new Pose(x, y, Math.toRadians(headingDegrees)), goal);
    }

    public static boolean hasAutoPose(ShooterConst.Goal goal) {
        return autoEndPose != null && autoGoal == goal;
    }

    public static Pose getAutoPose(ShooterConst.Goal goal) {
        return hasAutoPose(goal) ? autoEndPose : null;
    }

    public static void clearAutoPose() {
        autoEndPose = null;
        autoGoal = null;
    }
}
