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
import org.firstinspires.ftc.teamcode.subConstant.AutoConst;
import org.firstinspires.ftc.teamcode.subConstant.PoseStorage;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@Autonomous(name = "RedFar 30s", group = "32020 AUTO")
public class Red extends OpMode {
    private enum AutoState {
        DRIVE_TO_FIRST_AND_SHOOT,
        DRIVE_TO_SECOND_INTAKE,
        RETURN_TO_FIRST,
        WAIT_FOR_RETURN_SHOT,
        FINAL_FIRING,
        DRIVE_FOURTH_INTAKE,
        DRIVE_FIFTH,
        DRIVE_SIXTH_SPINUP,
        WAIT_FOR_THIRD_SHOT,
        THIRD_FIRING,
        DRIVE_SEVENTH,
        DRIVE_TO_GATE_INTAKE,
        WAIT_AT_GATE_INTAKE,
        WAIT_FOR_FORTH_SHOT,

        FORTH_FIRING,
        DONE
    }


    private static final double FIRST_SHOT_DELAY_SECONDS = 0.5;
    private static final double RETURN_SHOT_DELAY_SECONDS = 0.5;
    private static final double FIRING_TIME_SECONDS = 0.5;

    private static final double INTAKING_TIME_SECONDS = 1.5;

    private final ArtifactIntake artifactIntake = new ArtifactIntake();

    private final AutoConst autoConst = new AutoConst();


    private static final Pose START_POSE =
            new Pose(109, 133, Math.toRadians(90));
    private static final Pose FIRST_POSE = // 슈팅위치
            new Pose(96, 82, Math.toRadians(0));
    private static final Pose SECOND_POSE =
            new Pose(121, 82, Math.toRadians(0)); // 첫번째 줄 유물 먹는 위치
    //shoot
    private static final Pose THIRD_POSE =
            new Pose(117, 62, Math.toRadians(0)); // 두번째 줄 유물 먹는 위치
    private static final Pose THIRD_CURVE =
            new Pose(92, 56, Math.toRadians(0)); // 두번째 줄 유물 먹으러 가는 커브 체크포인트
    private static final Pose FORTH_POSE =
            new Pose(124, 65, Math.toRadians(0)); // 첫번째 게이트 오픈 위치
    private static final Pose FIFTH_POSE =
            new Pose(120, 60, Math.toRadians(45)); //
    // shoot
    private static final Pose SIXTH_POSE = // gate open
            new Pose(130, 60, Math.toRadians(30));

    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Follower follower;
    private PathChain firstPath,
            secondPath,
            thirdPath,
            forthPath,
            fifthPath,
            sixthPath,
            seventhPath,
            eighthPath,
            returnPath;
    private AutoState autoState = AutoState.DRIVE_TO_FIRST_AND_SHOOT;
    private ShotCalculator.ShotResult shotResult;
    private boolean firstShotStarted;
    private boolean firstShotComplete;

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
                .addPath(new BezierLine(FIRST_POSE, SECOND_POSE))
                .setLinearHeadingInterpolation(
                        FIRST_POSE.getHeading(), SECOND_POSE.getHeading())
                .build();

        thirdPath = follower.pathBuilder()
                .addPath(new BezierLine(SECOND_POSE, FIRST_POSE))
                .setLinearHeadingInterpolation(
                SECOND_POSE.getHeading(), FIRST_POSE.getHeading())
                .build();

        forthPath = follower.pathBuilder()
                .addPath(new BezierCurve(FIRST_POSE, THIRD_CURVE, THIRD_POSE))
                .setLinearHeadingInterpolation(
                FIRST_POSE.getHeading(), THIRD_POSE.getHeading())
                .build();

        fifthPath = follower.pathBuilder()
                .addPath(new BezierLine(THIRD_POSE, FORTH_POSE))
                .setLinearHeadingInterpolation(
                THIRD_POSE.getHeading(), FORTH_POSE.getHeading())
                .build();
        sixthPath = follower.pathBuilder()
                .addPath(new BezierLine(FORTH_POSE, FIRST_POSE))
                .setLinearHeadingInterpolation(
                FORTH_POSE.getHeading(), FIRST_POSE.getHeading())
                .build();
        seventhPath = follower.pathBuilder()
                .addPath(new BezierLine(FIRST_POSE, FIFTH_POSE))
                .setLinearHeadingInterpolation(
                FIRST_POSE.getHeading(), FIFTH_POSE.getHeading())
                .build();

        returnPath = follower.pathBuilder()
                .addPath(new BezierLine(FIFTH_POSE, SIXTH_POSE))
                .setLinearHeadingInterpolation(
                        FIFTH_POSE.getHeading(), SIXTH_POSE.getHeading())
                .build();


        eighthPath = follower.pathBuilder()
                .addPath(new BezierLine(SIXTH_POSE, FIRST_POSE))
                .setLinearHeadingInterpolation(
                        SIXTH_POSE.getHeading(), FIRST_POSE.getHeading())
                .build();




        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        shooter.spinUp();
        firstShotStarted = false;
        firstShotComplete = false;
        stateTimer.reset();
        follower.followPath(firstPath);
        autoState = AutoState.DRIVE_TO_FIRST_AND_SHOOT;
    }

    @Override
    public void loop() {
        follower.update();
        updateAutomaticAim();

        switch (autoState) {
            case DRIVE_TO_FIRST_AND_SHOOT:
                shooter.spinUp();

                if (!firstShotStarted
                        && stateTimer.seconds() >= FIRST_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    shooter.fire();
                    stateTimer.reset();
                    firstShotStarted = true;
                }

                if (firstShotStarted && !firstShotComplete) {
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                        firstShotComplete = true;
                        artifactIntake.setState(ArtifactIntake.State.IDLE);
                    }
                } else {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                }

                if (firstShotComplete && !follower.isBusy()) {
                    shooter.stop();
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(secondPath);
                    autoState = AutoState.DRIVE_TO_SECOND_INTAKE;
                }
                break;

            case DRIVE_TO_SECOND_INTAKE:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (!follower.isBusy()) {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    shooter.spinUp();
                    follower.followPath(thirdPath);
                    autoState = AutoState.RETURN_TO_FIRST;
                }
                break;

            case RETURN_TO_FIRST:
                shooter.spinUp();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.WAIT_FOR_RETURN_SHOT;
                }
                break;

            case WAIT_FOR_RETURN_SHOT:
                shooter.spinUp();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (stateTimer.seconds() >= RETURN_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    shooter.fire();
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    stateTimer.reset();
                    autoState = AutoState.FINAL_FIRING;
                }
                break;

            case FINAL_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    shooter.stop();
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(forthPath);
                    autoState = AutoState.DRIVE_FOURTH_INTAKE;
                }
                break;

            case DRIVE_FOURTH_INTAKE:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (!follower.isBusy()) {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    follower.followPath(fifthPath);
                    autoState = AutoState.DRIVE_FIFTH;
                }
                break;

            case DRIVE_FIFTH:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (!follower.isBusy()) {
                    shooter.spinUp();
                    follower.followPath(sixthPath);
                    autoState = AutoState.DRIVE_SIXTH_SPINUP;
                }
                break;

            case DRIVE_SIXTH_SPINUP:
                shooter.spinUp();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (!follower.isBusy()) {
                    autoState = AutoState.WAIT_FOR_THIRD_SHOT;
                }
                break;

            case WAIT_FOR_THIRD_SHOT:
                shooter.spinUp();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (shooter.getState() == Shooter.State.READY) {
                    shooter.fire();
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    stateTimer.reset();
                    autoState = AutoState.THIRD_FIRING;
                }
                break;

            case THIRD_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    shooter.stop();
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    follower.followPath(seventhPath);
                    autoState = AutoState.DRIVE_SEVENTH;
                }
                break;

            case DRIVE_SEVENTH:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (!follower.isBusy())  {
                    follower.followPath(returnPath);
                    autoState = AutoState.DRIVE_TO_GATE_INTAKE;
                }
                break;

            case DRIVE_TO_GATE_INTAKE:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_INTAKE;
                }
                break;

            case WAIT_AT_GATE_INTAKE:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (stateTimer.seconds() >= INTAKING_TIME_SECONDS) {
                    follower.followPath(eighthPath);
                    autoState = AutoState.WAIT_FOR_FORTH_SHOT;
                }
                break;

            case WAIT_FOR_FORTH_SHOT:
                shooter.spinUp();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (!follower.isBusy()
                        && shooter.getState() == Shooter.State.READY) {
                    shooter.fire();
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    stateTimer.reset();
                    autoState = AutoState.FORTH_FIRING;
                }
                break;

            case FORTH_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (!follower.isBusy() && stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    shooter.stop();
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(seventhPath);
                    autoState = AutoState.DRIVE_SEVENTH;
                }
                break;


            case DONE:
            default:
                shooter.stop();
                artifactIntake.setState(ArtifactIntake.State.IDLE);
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
