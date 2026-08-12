package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.ArtifactIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShotCalculator;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subConstant.FieldConst;
import org.firstinspires.ftc.teamcode.subConstant.PoseStorage;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

public abstract class TeleOpTest extends OpMode {

    private Follower follower;
    private ArtifactIntake artifactIntake = new ArtifactIntake();
    private Shooter shooter = new Shooter();
    private Turret turret = new Turret();
    private ShooterConst.Goal activeGoal;
    private double goalX;
    private double goalY;
    private Pose startingPose;
    private double driverHeadingDegrees;
    private boolean usingAutoPose;
    private boolean previousPoseResetCombo;
    private boolean previousTurretResetCombo;

    protected abstract ShooterConst.Goal getGoal();

    @Override
    public void init() {
        activeGoal = getGoal();
        goalX = activeGoal == ShooterConst.Goal.RED
                ? ShooterConst.RED_GOAL_X : ShooterConst.BLUE_GOAL_X;
        goalY = activeGoal == ShooterConst.Goal.RED
                ? ShooterConst.RED_GOAL_Y : ShooterConst.BLUE_GOAL_Y;

        double startX = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_START_X : FieldConst.BLUE_START_X;
        double startY = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_START_Y : FieldConst.BLUE_START_Y;
        double startHeading = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_START_HEADING_DEGREES
                : FieldConst.BLUE_START_HEADING_DEGREES;
        driverHeadingDegrees = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_DRIVER_HEADING_DEGREES
                : FieldConst.BLUE_DRIVER_HEADING_DEGREES;
        startingPose = new Pose(
                startX,
                startY,
                Math.toRadians(startHeading));
        Pose autoEndPose = PoseStorage.getAutoPose(activeGoal);
        if (autoEndPose != null) {
            startingPose = autoEndPose;
            usingAutoPose = true;
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        artifactIntake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
    }

    @Override
    public void start() {
        follower.setPose(startingPose);
        follower.startTeleopDrive();
        follower.update();
        previousPoseResetCombo = false;
        previousTurretResetCombo = false;
    }

    @Override
    public void loop(){


        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                false,
                Math.toRadians(driverHeadingDegrees));
        follower.update();

        boolean poseResetCombo = gamepad1.start && gamepad1.x;
        boolean turretResetCombo = gamepad1.start && gamepad1.y;

        if (poseResetCombo && !previousPoseResetCombo) {
            follower.setPose(getRecoveryPose());
            PoseStorage.clearAutoPose();
            usingAutoPose = false;
        }
        if (turretResetCombo && !previousTurretResetCombo) {
            turret.resetAngleTracking();
        }

        previousPoseResetCombo = poseResetCombo;
        previousTurretResetCombo = turretResetCombo;

        if (gamepad1.left_bumper) {
            artifactIntake.setState(ArtifactIntake.State.INTAKING);
        } else if (gamepad1.right_bumper) {
            artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
        } else {
            artifactIntake.setState(ArtifactIntake.State.IDLE);
        }
        artifactIntake.update();

        Pose position = follower.getPose();
        Vector robotVelocity = follower.getVelocity();
        double robotVelocityX = robotVelocity.getXComponent();
        double robotVelocityY = robotVelocity.getYComponent();

        ShotCalculator.ShotResult shotResult = shooter.aimAt(
                position,
                goalX,
                goalY,
                robotVelocityX,
                robotVelocityY);


        if(gamepad2.dpadUpWasPressed()){
            ShooterConst.SHOOTER_POWER_RATIO_MAX += 0.1;
        }else if(gamepad2.dpadDownWasPressed()){
            ShooterConst.SHOOTER_POWER_RATIO_MAX -= 0.1;
        }

        if(gamepad2.yWasPressed()){
            ShooterConst.SHOOTER_POWER_RATIO += 0.1;
        }else if(gamepad2.dpadDownWasPressed()){
            ShooterConst.SHOOTER_POWER_RATIO -= 0.1;
        }


        // 주행 중에도 현재 위치와 헤딩을 기준으로 골대를 계속 추적한다.
        // ShotCalculator의 보정각을 적용해 로봇의 이동 속도까지 반영한다.
        if (shotResult != null) {
            turret.trackPoint(position, goalX, goalY, shotResult.turretOffset);
        } else {
            turret.trackPoint(position, goalX, goalY);
        }

        if (gamepad1.right_trigger_pressed) {
            shooter.spinUp();
            if (gamepad1.b) {
                shooter.fire();
            }
        } else {
            shooter.stop();
        }
        shooter.update();


        if (position != null) {
            telemetry.addData("Heading (Degrees)", Math.toDegrees(position.getHeading()));
            telemetry.addData("Heading (Radians)", position.getHeading());
            telemetry.addData("X", position.getX());
            telemetry.addData("Y", position.getY());
            telemetry.addData("velocityX", robotVelocityX);
            telemetry.addData("velocityY", robotVelocityY);
        }
        telemetry.addLine("==========TUNING ZONE=========");
        telemetry.addData("Shooter Power Ratio Close", ShooterConst.SHOOTER_POWER_RATIO);
        telemetry.addData("Shooter Power Ratio Max", ShooterConst.SHOOTER_POWER_RATIO_MAX);
        telemetry.addLine("=================================");

        telemetry.addData("curVelo", shooter.ShooterLeft.getVelocity());
        telemetry.addData("targetVelo", shooter.getTargetVelocity());
        telemetry.addData("shooterPowerRatio", shooter.getAppliedPowerRatio());
        telemetry.addData("intakeState", artifactIntake.getState());
        telemetry.addData("shooterState", shooter.getState());
        telemetry.addData("activeGoal", activeGoal);
        telemetry.addData("usingAutoPose", usingAutoPose);
        telemetry.addData("driverHeadingDeg", driverHeadingDegrees);
        telemetry.addData("goalX", goalX);
        telemetry.addData("goalY", goalY);
        telemetry.addData("turretTargetDeg", turret.getTargetAngleDegrees());
        telemetry.addData("turretCurrentDeg", turret.getCurrentAngleDegrees());
        telemetry.addData("turretErrorDeg", turret.getAngleErrorDegrees());
        telemetry.addData("turretPower", turret.getServoPower());
        telemetry.addData("turretEncoderVoltage", turret.getEncoderVoltage());
        if (shotResult != null) {
            telemetry.addData("shotDistance", shotResult.distanceToGoal);
            telemetry.addData("turretMoveOffsetDeg",
                    Math.toDegrees(shotResult.turretOffset));
            telemetry.addData("hoodAngleDeg", Math.toDegrees(shotResult.hoodAngle));
        }
        telemetry.update();
    }


    private Pose getRecoveryPose() {
        double recoveryX = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_RECOVERY_X : FieldConst.BLUE_RECOVERY_X;
        double recoveryY = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_RECOVERY_Y : FieldConst.BLUE_RECOVERY_Y;
        double recoveryHeading = activeGoal == ShooterConst.Goal.RED
                ? FieldConst.RED_RECOVERY_HEADING_DEGREES
                : FieldConst.BLUE_RECOVERY_HEADING_DEGREES;

        return new Pose(
                recoveryX,
                recoveryY,
                Math.toRadians(recoveryHeading));
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            follower.update();
        }
        turret.stop();
        shooter.stop();
        shooter.update();
    }
}
