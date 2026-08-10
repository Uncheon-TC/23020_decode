package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.ArtifactIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShotCalculator;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subConstant.PoseStorage;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@Autonomous(name = "AutoBlueFar1 60s", group = "32020 AUTO")
public class BlueFar1 extends OpMode {
    private enum AutoState {
        DRIVE_TO_FIRST,
        DRIVE_TO_SECOND,
        DRIVE_TO_THIRD,
    }

    private static final Pose START_POSE =
            new Pose(95, 9.7, Math.toRadians(0));
    private static final Pose FIRST_POSE =
            new Pose(130, 9.7, Math.toRadians(0));
    private static final Pose FIRST_CURVE =
            new Pose(127, 25, Math.toRadians(0));
    private static final Pose SECOND_POSE =
            new Pose(128, 50, Math.toRadians(0));


    private static final double SHOT_DELAY_SECONDS = 0.5;
    private static final double FIRING_TIME_SECONDS = 0.5;

    private final ArtifactIntake artifactIntake = new ArtifactIntake();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Follower follower;
    private PathChain firstPath;
    private PathChain secondPath;
    private PathChain returnPath;
    private AutoState autoState = AutoState.DRIVE_TO_FIRST;
    private ShotCalculator.ShotResult shotResult;
    private boolean ShotStarted, ShotCompleted;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        artifactIntake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, FIRST_POSE))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(), FIRST_POSE.getHeading())
                .build();

        secondPath = follower.pathBuilder()
                .addPath(new BezierCurve(FIRST_POSE, FIRST_CURVE, SECOND_POSE))
                .setLinearHeadingInterpolation(
                        FIRST_POSE.getHeading(), SECOND_POSE.getHeading())
                .build();

        returnPath = follower.pathBuilder()
                .addPath(new BezierLine(SECOND_POSE, START_POSE))
                .setLinearHeadingInterpolation(
                        SECOND_POSE.getHeading(), START_POSE.getHeading())
                .build();

        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        shooter.spinUp();
        ShotStarted = false;
        ShotCompleted = false;
        stateTimer.reset();
        follower.followPath(firstPath);
        autoState = AutoState.DRIVE_TO_FIRST;
    }

    @Override
    public void loop() {
        follower.update();
        updateAutomaticAim();

        switch (autoState) {
            case DRIVE_TO_FIRST:

                artifactIntake.setState(ArtifactIntake.State.INTAKING);

                if(!follower.isBusy()){
                    follower.followPath(secondPath);
                    autoState = AutoState.DRIVE_TO_SECOND;
                }

            case DRIVE_TO_SECOND:

                artifactIntake.setState(ArtifactIntake.State.INTAKING);

                if (!follower.isBusy()) {
                    shooter.spinUp();
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    stateTimer.reset();;
                    follower.followPath(returnPath);
                    autoState = AutoState.DRIVE_TO_THIRD;
                }

                break;


            case DRIVE_TO_THIRD:

                if(!follower.isBusy() && shooter.getState() == Shooter.State.READY
                        && stateTimer.seconds() >= SHOT_DELAY_SECONDS){
                    shooter.fire();
                    stateTimer.reset();
                    ShotStarted = true;
                }


                if (ShotStarted && !ShotCompleted) {
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    if (stateTimer.seconds() >= SHOT_DELAY_SECONDS) {
                        ShotCompleted = true;
                        artifactIntake.setState(ArtifactIntake.State.IDLE);
                    }
                } else {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                }

                if(ShotCompleted) {
                    shooter.stop();
                    stateTimer.reset();
                    autoState = AutoState.DRIVE_TO_FIRST;
                    ShotStarted = false;
                    ShotCompleted = false;

                }

        }

        shooter.update();
        artifactIntake.update();
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        updateTelemetry();
    }

    @Override
    public void stop() {
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        shooter.stop();
        shooter.update();
        artifactIntake.setState(ArtifactIntake.State.IDLE);
        artifactIntake.update();
        turret.stop();
    }

    private void updateAutomaticAim() {
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        shotResult = shooter.aimAt(
                pose,
                ShooterConst.RED_GOAL_X,
                ShooterConst.RED_GOAL_Y,
                velocity.getXComponent(),
                velocity.getYComponent());

        if (shotResult != null) {
            turret.trackPoint(
                    pose,
                    ShooterConst.RED_GOAL_X,
                    ShooterConst.RED_GOAL_Y,
                    shotResult.turretOffset);
        } else {
            turret.trackPoint(
                    pose,
                    ShooterConst.RED_GOAL_X,
                    ShooterConst.RED_GOAL_Y);
        }
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("autoState", autoState);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("headingDeg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("pathBusy", follower.isBusy());
        telemetry.addData("shooterState", shooter.getState());
        telemetry.addData("shooterVelocity", shooter.ShooterLeft.getVelocity());
        telemetry.addData("shooterTarget", shooter.getTargetVelocity());
        telemetry.addData("shooterPowerRatio", shooter.getAppliedPowerRatio());
        telemetry.addData("intakeState", artifactIntake.getState());
        telemetry.addData("turretTargetDeg", turret.getTargetAngleDegrees());
        telemetry.addData("turretCurrentDeg", turret.getCurrentAngleDegrees());
        telemetry.addData("turretPower", turret.getServoPower());
        if (shotResult != null) {
            telemetry.addData("shotDistance", shotResult.distanceToGoal);
            telemetry.addData("hoodAngleDeg", Math.toDegrees(shotResult.hoodAngle));
        }
        telemetry.update();
    }
}
