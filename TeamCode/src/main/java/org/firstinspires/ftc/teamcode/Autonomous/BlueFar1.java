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

@Autonomous(name = "AutoBlueFar 60s", group = "32020 AUTO")
public class BlueFar1 extends OpMode {
    private enum AutoState {

        //-------canceled just in case--------
        SHOOT_PRELOAD,

        //------------------------------------

        // ---- vertical ball, first pass ----
        DRIVE_TO_PATH3,
        DRIVE_TO_PATH4,
        DRIVE_TO_PATH5,
        DRIVE_TO_PATH6,
        SHOOT_AT_PATH6,

        // ---- vertical ball, repeat ----
        DRIVE_TO_PATH7,
        DRIVE_TO_PATH8,
        DRIVE_TO_PATH9,
        DRIVE_TO_PATH10,
        SHOOT_AT_PATH10,

        // ---- secret tunnel ----
        DRIVE_TO_PATH11,
        DRIVE_TO_PATH12,
        SHOOT_AT_PATH12,

    }

    public static final Pose STARTING_POSE =
            new Pose(48, 15, Math.toRadians(180));
    //=====================REPEAT========================
// vertical ball
    private static final Pose PATH3_POSE =
            new Pose(14, 9, Math.toRadians(180)); // collects artifact
    private static final Pose PATH4_POSE =
            new Pose(19, 9, Math.toRadians(180)); // goes back and forth
    private static final Pose PATH5_POSE =
            new Pose(14, 9, Math.toRadians(180)); // collects again
    private static final Pose PATH6_POSE =
            new Pose(48, 9, Math.toRadians(180)); // shooting
    // repeat vertical ball
    private static final Pose PATH7_POSE =
            new Pose(14, 9, Math.toRadians(180)); // collects artifact (r)
    private static final Pose PATH8_POSE =
            new Pose(19, 9, Math.toRadians(180)); // goes back and forth
    private static final Pose PATH9_POSE =
            new Pose(14, 9, Math.toRadians(180)); // collects again
    private static final Pose PATH10_POSE =
            new Pose(48, 9, Math.toRadians(180)); // shooting
    // goes to secret tunnel
    private static final Pose PATH11_POSE =
            new Pose(14, 33, Math.toRadians(180)); // collects artifact
    private static final Pose PATH12_POSE =
            new Pose(48, 15, Math.toRadians(180)); // shooting


    private static final double SHOT_DELAY_SECONDS  = 0.5;
    private static final double FIRING_TIME_SECONDS = 0.5;

    private final ArtifactIntake artifactIntake = new ArtifactIntake();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Follower follower;
    private PathChain thirdPath, fourthPath, fifthPath, sixthPath, seventhPath, eighthPath, ninthPath, tenthPath, eleventhPath, twelfthPath;
    private AutoState autoState = AutoState.SHOOT_PRELOAD;
    private ShotCalculator.ShotResult shotResult;

    // sub-state shared by every SHOOT_AT_* state
    private boolean shotStarted, shotCompleted;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);

        artifactIntake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        // ---- vertical ball, first pass ----
        thirdPath = follower.pathBuilder() // -> collect artifact
                .addPath(new BezierLine(STARTING_POSE, PATH3_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fourthPath = follower.pathBuilder() // -> goes back and forth
                .addPath(new BezierLine(PATH3_POSE, PATH4_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fifthPath = follower.pathBuilder() // -> collects again
                .addPath(new BezierLine(PATH4_POSE, PATH5_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        sixthPath = follower.pathBuilder() // -> shooting
                .addPath(new BezierLine(PATH5_POSE, PATH6_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // ---- repeat vertical ball ----
        seventhPath = follower.pathBuilder() // -> collect artifact (r)
                .addPath(new BezierLine(PATH6_POSE, PATH7_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        eighthPath = follower.pathBuilder() // -> goes back and forth
                .addPath(new BezierLine(PATH7_POSE, PATH8_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ninthPath = follower.pathBuilder() // -> collects again
                .addPath(new BezierLine(PATH8_POSE, PATH9_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        tenthPath = follower.pathBuilder() // -> shooting
                .addPath(new BezierLine(PATH9_POSE, PATH10_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // ---- goes to secret tunnel ----
        eleventhPath = follower.pathBuilder() // -> collect artifact
                .addPath(new BezierLine(PATH10_POSE, PATH11_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        twelfthPath = follower.pathBuilder() // -> shooting
                .addPath(new BezierLine(PATH11_POSE, PATH12_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        follower.setPose(STARTING_POSE);
        stateTimer.reset();

        artifactIntake.setState(ArtifactIntake.State.INTAKING);
        follower.followPath(thirdPath);
        autoState = AutoState.DRIVE_TO_PATH3;
    }

    @Override
    public void loop() {
        follower.update();
        updateAutomaticAim();

        switch (autoState) {

            //==================CANCELED========================
            // ---- shoot the preloaded artifacts before moving ----
            case SHOOT_PRELOAD:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(thirdPath);
                    autoState = AutoState.DRIVE_TO_PATH3;
                }
                break;

            //==================================================

            // ---- vertical ball, first pass ----
            case DRIVE_TO_PATH3: // collects artifact
                if (!follower.isBusy()) {
                    follower.followPath(fourthPath);
                    autoState = AutoState.DRIVE_TO_PATH4;
                }
                break;

            case DRIVE_TO_PATH4: // goes back and forth
                if (!follower.isBusy()) {
                    follower.followPath(fifthPath);
                    autoState = AutoState.DRIVE_TO_PATH5;
                }
                break;

            case DRIVE_TO_PATH5: // collects again
                if (!follower.isBusy()) {
                    follower.followPath(sixthPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_PATH6;
                }
                break;

            case DRIVE_TO_PATH6: // -> shooting
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_PATH6;
                }
                break;

            case SHOOT_AT_PATH6:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(seventhPath);
                    autoState = AutoState.DRIVE_TO_PATH7;
                }
                break;

            // ---- vertical ball, repeat ----
            case DRIVE_TO_PATH7: // collects artifact (r)
                if (!follower.isBusy()) {
                    follower.followPath(eighthPath);
                    autoState = AutoState.DRIVE_TO_PATH8;
                }
                break;

            case DRIVE_TO_PATH8: // goes back and forth
                if (!follower.isBusy()) {
                    follower.followPath(ninthPath);
                    autoState = AutoState.DRIVE_TO_PATH9;
                }
                break;

            case DRIVE_TO_PATH9: // collects again
                if (!follower.isBusy()) {
                    follower.followPath(tenthPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_PATH10;
                }
                break;

            case DRIVE_TO_PATH10: // -> shooting
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_PATH10;
                }
                break;

            case SHOOT_AT_PATH10:
                if (updateShotSequence()) {
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(eleventhPath);
                    autoState = AutoState.DRIVE_TO_PATH11;
                }
                break;

            // ---- secret tunnel ----
            case DRIVE_TO_PATH11: // collects artifact
                if (!follower.isBusy()) {
                    follower.followPath(twelfthPath);
                    shooter.spinUp();
                    autoState = AutoState.DRIVE_TO_PATH12;
                }
                break;

            case DRIVE_TO_PATH12: // -> shooting
                if (!follower.isBusy()) {
                    beginShotSequence();
                    autoState = AutoState.SHOOT_AT_PATH12;
                }
                break;

            case SHOOT_AT_PATH12:
                if (updateShotSequence()) {
                    //repeat
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    follower.followPath(thirdPath);
                    autoState = AutoState.DRIVE_TO_PATH3;
                }
                break;

        }


        shooter.update();
        artifactIntake.update();
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.BLUE);
        updateTelemetry();
    }

    @Override
    public void stop() {
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.BLUE);
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
                ShooterConst.BLUE_GOAL_X,
                ShooterConst.BLUE_GOAL_Y,
                velocity.getXComponent(),
                velocity.getYComponent());

        if (shotResult != null) {
            turret.trackPoint(pose, ShooterConst.BLUE_GOAL_X, ShooterConst.BLUE_GOAL_Y, shotResult.turretOffset);
        } else {
            turret.trackPoint(pose, ShooterConst.BLUE_GOAL_X, ShooterConst.BLUE_GOAL_Y);
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