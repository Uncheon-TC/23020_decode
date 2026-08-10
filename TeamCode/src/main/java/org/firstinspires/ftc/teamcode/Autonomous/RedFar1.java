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

@Autonomous(name = "AutoRedFar 60s", group = "32020 AUTO")
public class RedFar1 extends OpMode {
    private enum AutoState {

        SHOOT_AT_FIRST,
        DRIVE_TO_FIRST,
        DRIVE_TO_SECOND,
        SHOOT_AT_SECOND,
        DRIVE_TO_THIRD,
        DRIVE_TO_FOURTH,
        SHOOT_AT_FOURTH,
        DRIVE_TO_FIFTH,
        DRIVE_TO_SIXTH,
        SHOOT_AT_SIXTH,
        DRIVE_TO_SEVENTH,
        DRIVE_TO_EIGHTH,
        SHOOT_AT_EIGHTH,
    }

    private static final Pose START_POSE   = new Pose(96, 9, Math.toRadians(0));
    private static final Pose FIRST_POSE   = new Pose(130, 9.7, Math.toRadians(0));
    private static final Pose SECOND_POSE  = new Pose(96, 9, Math.toRadians(0));
    private static final Pose SECOND_CURVE = new Pose(101, 38, Math.toRadians(0));
    private static final Pose THIRD_POSE   = new Pose(126, 34, Math.toRadians(0));
    private static final Pose FOURTH_POSE  = new Pose(96, 15, Math.toRadians(0));
    private static final Pose FIFTH_POSE   = new Pose(130, 9.7, Math.toRadians(0));
    private static final Pose SIXTH_POSE = new Pose(96, 15, Math.toRadians(0));
    private static final Pose SEVENTH_POSE   = new Pose(130, 42, Math.toRadians(30));
    private static final Pose EIGHTH_POSE = new Pose(96, 15, Math.toRadians(0));
    private static final double SHOT_DELAY_SECONDS  = 0.5;
    private static final double FIRING_TIME_SECONDS = 0.5;

    private final ArtifactIntake artifactIntake = new ArtifactIntake();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Follower follower;
    private PathChain firstPath, secondPath, thirdPath, fourthPath, fifthPath, sixthPath, seventhPath, eighthPath;
    private AutoState autoState = AutoState.DRIVE_TO_FIRST;
    private ShotCalculator.ShotResult shotResult;

    // sub-state shared by every SHOOT_AT_* state
    private boolean shotStarted, shotCompleted;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        artifactIntake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, FIRST_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), FIRST_POSE.getHeading())
                .build();

        secondPath = follower.pathBuilder()
                .addPath(new BezierLine(FIRST_POSE, SECOND_POSE))
                .setConstantHeadingInterpolation(SECOND_POSE.getHeading())
                .build();

        thirdPath = follower.pathBuilder()
                .addPath(new BezierCurve(SECOND_POSE, SECOND_CURVE, THIRD_POSE))
                .setConstantHeadingInterpolation(THIRD_POSE.getHeading())
                .build();

        fourthPath = follower.pathBuilder()
                .addPath(new BezierLine(THIRD_POSE, FOURTH_POSE))
                .setConstantHeadingInterpolation(FOURTH_POSE.getHeading())
                .build();

        fifthPath = follower.pathBuilder()
                .addPath(new BezierLine(FOURTH_POSE, FIFTH_POSE))
                .setLinearHeadingInterpolation(FOURTH_POSE.getHeading(), FIFTH_POSE.getHeading())
                .build();

        sixthPath = follower.pathBuilder()
                .addPath(new BezierLine(FIFTH_POSE, SIXTH_POSE))
                .setLinearHeadingInterpolation(FIFTH_POSE.getHeading(), SIXTH_POSE.getHeading())
                .build();

        seventhPath = follower.pathBuilder()
                .addPath(new BezierLine(SIXTH_POSE, SEVENTH_POSE))
                .setLinearHeadingInterpolation(SIXTH_POSE.getHeading(), SEVENTH_POSE.getHeading())
                .build();
        eighthPath = follower.pathBuilder()
                .addPath(new BezierLine(SEVENTH_POSE, EIGHTH_POSE))
                .setLinearHeadingInterpolation(SEVENTH_POSE.getHeading(), EIGHTH_POSE.getHeading())
                .build();

        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        stateTimer.reset();

        artifactIntake.setState(ArtifactIntake.State.INTAKING);
        follower.followPath(firstPath);
        autoState = AutoState.SHOOT_AT_FIRST;
    }

    @Override
    public void loop() {
        follower.update();
        updateAutomaticAim();

        switch (autoState) {

            case SHOOT_AT_FIRST:
                shooter.spinUp();
                beginShotSequence();
                if(updateShotSequence()){
                    autoState = AutoState.DRIVE_TO_FIRST;
                }

            case DRIVE_TO_FIRST:
                if (!follower.isBusy()) {
                    follower.followPath(secondPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_SECOND;
                }
                break;

            case DRIVE_TO_SECOND:
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_SECOND;
                }
                break;

            case SHOOT_AT_SECOND:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(thirdPath);
                    autoState = AutoState.DRIVE_TO_THIRD;
                }
                break;

            case DRIVE_TO_THIRD:
                if (!follower.isBusy()) {
                    follower.followPath(fourthPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_FOURTH;
                }
                break;

            case DRIVE_TO_FOURTH:
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_FOURTH;
                }
                break;

            case SHOOT_AT_FOURTH:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(fifthPath);
                    autoState = AutoState.DRIVE_TO_FIFTH;
                }
                break;

            case DRIVE_TO_FIFTH:
                if (!follower.isBusy()) {
                    follower.followPath(sixthPath);
                    autoState = AutoState.DRIVE_TO_SIXTH;
                }
                break;

            case DRIVE_TO_SIXTH:
                if (!follower.isBusy()) {
                    shooter.spinUp();
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_SIXTH;
                }
                break;

            case SHOOT_AT_SIXTH:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(seventhPath);
                    autoState = AutoState.DRIVE_TO_SEVENTH;
                }
                break;

            case DRIVE_TO_SEVENTH:
                if (!follower.isBusy()) {
                    follower.followPath(eighthPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_EIGHTH;
                }
                break;

            case DRIVE_TO_EIGHTH:
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_EIGHTH;
                }

            case SHOOT_AT_EIGHTH:
                if (updateShotSequence()) {
                    // repeat
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(fifthPath);
                    autoState = AutoState.DRIVE_TO_FIFTH;
                }
                break;
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


    private void beginShotSequence() {
        stateTimer.reset();
        shotStarted = false;
        shotCompleted = false;
    }

    private boolean updateShotSequence() {
        if (!shotStarted
                && stateTimer.seconds() >= SHOT_DELAY_SECONDS
                && shooter.getState() == Shooter.State.READY) {
            shooter.fire();
            stateTimer.reset();
            shotStarted = true;
        }

        if (shotStarted && !shotCompleted) {
            artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
            if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                shotCompleted = true;
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                shooter.stop();
            }
        }

        return shotCompleted;
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
            turret.trackPoint(pose, ShooterConst.RED_GOAL_X, ShooterConst.RED_GOAL_Y, shotResult.turretOffset);
        } else {
            turret.trackPoint(pose, ShooterConst.RED_GOAL_X, ShooterConst.RED_GOAL_Y);
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