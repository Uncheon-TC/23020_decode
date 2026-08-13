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

@Autonomous(name = "AutoRedFarT 60s", group = "32020 AUTO")
public class RedFarT extends OpMode {
    // 자율주행의 현재 진행 단계를 나타낸다.
    private enum AutoState {
        WAIT_FOR_PRELOAD_SHOT,       // 시작 위치에서 프리로드 발사 준비
        PRELOAD_FIRING,              // 프리로드 3개 발사
        DRIVE_PATH1_INTAKE,          // 경로 1: 아래 로딩존 수집
        DRIVE_PATH2_SPINUP,          // 경로 2: 슈팅 위치 복귀
        WAIT_FOR_PATH2_SHOT,         // 경로 2 도착 후 발사 준비
        PATH2_FIRING,                // 경로 2 도착 후 발사
        DRIVE_PATH3_INTAKE,          // 경로 3: 아래 로딩존 수집
        DRIVE_PATH4_SPINUP,          // 경로 4: 슈팅 위치 복귀
        WAIT_FOR_PATH4_SHOT,         // 경로 4 도착 후 발사 준비
        PATH4_FIRING,                // 경로 4 도착 후 발사
        DRIVE_PATH5_INTAKE,          // 경로 5: 위 로딩존 수집
        DRIVE_PATH6_SPINUP,          // 경로 6: 슈팅 위치 복귀
        WAIT_FOR_PATH6_SHOT,         // 경로 6 도착 후 발사 준비
        PATH6_FIRING,                // 경로 6 도착 후 발사 및 반복
        PARKING,                     // 58초부터 주차 위치로 이동
        DONE                         // 주차 완료 또는 60초 도달 후 정지
    }

    // 시작 위치에서 프리로드를 발사하기 전에 기다리는 시간이다.
    private static final double PRELOAD_SHOT_DELAY_SECONDS = 0.5;
    // 이동 경로가 끝난 뒤 두 번째 이후 발사 전에 기다리는 시간이다.
    private static final double POST_ARRIVAL_SHOT_DELAY_SECONDS = 0.2;
    // 세 개의 유물을 슈터로 밀어내는 아웃테이크 유지시간이다.
    private static final double FIRING_TIME_SECONDS = 0.5;
    // 벽이나 유물에 걸렸을 때 한 경로에서 무한히 머무르지 않게 하는 제한시간이다.
    private static final double PATH_TIMEOUT_SECONDS = 2.5;
    // 마지막 2초를 주차에 사용하기 위해 주차를 시작하는 시간이다.
    private static final double PARK_START_SECONDS = 58.0;
    // 이 시간이 되면 주차 성공 여부와 관계없이 OpMode를 종료한다.
    private static final double AUTO_END_SECONDS = 60.0;

    // 시작 및 반복 슈팅 위치이다.
    private static final Pose START_POSE =
            new Pose(96, 9, Math.toRadians(0));
    // 경로 1의 끝: 아래쪽 로딩존이다.
    private static final Pose LOWER_LOADING_POSE =
            new Pose(131, 15, Math.toRadians(0));
    // 경로 2, 4, 6의 끝: 공통 슈팅 위치이다.
    private static final Pose SHOOTING_POSE =
            new Pose(94, 18, Math.toRadians(0));
    // 경로 5 곡선의 모양을 결정하는 제어점이다.
    private static final Pose UPPER_LOADING_CONTROL_POSE =
            new Pose(104, 45, Math.toRadians(0));
    // 경로 5의 끝: 위쪽 로딩존이다.
    private static final Pose UPPER_LOADING_POSE =
            new Pose(132, 36, Math.toRadians(0));
    // 58초부터 이동할 최종 주차 위치이다.
    private static final Pose PARK_POSE =
            new Pose(107, 22, Math.toRadians(0));

    private final ArtifactIntake artifactIntake = new ArtifactIntake();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();

    // 발사 대기시간과 발사 유지시간을 측정한다.
    private final ElapsedTime stateTimer = new ElapsedTime();
    // 각 경로의 제한시간을 독립적으로 측정한다.
    private final ElapsedTime pathTimer = new ElapsedTime();
    // START 버튼을 누른 뒤의 전체 자율주행 시간을 측정한다.
    private final ElapsedTime autoTimer = new ElapsedTime();

    private Follower follower;
    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;

    private AutoState autoState = AutoState.WAIT_FOR_PRELOAD_SHOT;
    private ShotCalculator.ShotResult shotResult;
    private boolean lastPathTimedOut;
    private boolean parkingStarted;

    @Override
    public void init() {
        // Pedro Follower와 Pinpoint 로컬라이저를 생성하고 시작 Pose를 설정한다.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // 슈터, 터렛, 인테이크 하드웨어를 연결한다.
        artifactIntake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        // 경로 1: 시작 위치에서 아래 로딩존으로 이동한다.
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, LOWER_LOADING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 2: 아래 로딩존에서 슈팅 위치로 복귀한다.
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(LOWER_LOADING_POSE, SHOOTING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 3: 슈팅 위치에서 아래 로딩존으로 다시 이동한다.
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTING_POSE, LOWER_LOADING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 4: 아래 로딩존에서 슈팅 위치로 복귀한다.
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(LOWER_LOADING_POSE, SHOOTING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 5: 슈팅 위치에서 곡선을 따라 위 로딩존으로 이동한다.
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        SHOOTING_POSE,
                        UPPER_LOADING_CONTROL_POSE,
                        UPPER_LOADING_POSE))
                .setTangentHeadingInterpolation()
                .build();

        // 경로 6: 위 로딩존에서 같은 곡선을 반대로 따라 슈팅 위치로 복귀한다.
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        UPPER_LOADING_POSE,
                        UPPER_LOADING_CONTROL_POSE,
                        SHOOTING_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 이전 자율주행에서 저장된 Pose를 지운다.
        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        // START 버튼을 누른 순간 로컬라이저의 위치를 시작 Pose로 확정한다.
        follower.setPose(START_POSE);

        // 프리로드 발사 준비를 위해 즉시 슈터를 가속한다.
        // 이후 stop() 전까지 shooter.stop()을 호출하지 않아 계속 회전시킨다.
        shooter.spinUp();
        artifactIntake.setState(ArtifactIntake.State.IDLE);

        // 프리로드 발사 전 대기시간을 측정한다.
        stateTimer.reset();
        // 58초 주차와 60초 종료를 판단할 전체 타이머를 시작한다.
        autoTimer.reset();
        parkingStarted = false;
        autoState = AutoState.WAIT_FOR_PRELOAD_SHOT;
    }

    @Override
    public void loop() {
        // 현재 Pose와 속도를 갱신하고 경로를 추종한다.
        follower.update();

        // 58초가 되면 어떤 상태에 있더라도 현재 작업을 중단하고 주차를 시작한다.
        if (!parkingStarted && autoTimer.seconds() >= PARK_START_SECONDS) {
            beginParking();
        }

        // 주차가 시작된 이후에는 슈팅 상태머신을 더 이상 실행하지 않는다.
        if (parkingStarted) {
            updateParking();
            return;
        }

        // 텔레옵과 동일한 계산으로 슈터와 터렛 목표값을 갱신한다.
        updateAutomaticAim();
        // IDLE 상태가 된 경우에도 슈터가 다시 가속되도록 요청한다.
        // READY 또는 FIRING 상태에서는 현재 상태를 그대로 유지한다.
        shooter.spinUp();

        switch (autoState) {
            case WAIT_FOR_PRELOAD_SHOT:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 시작 후 0.5초가 지났고 슈터가 목표속도에 도달했을 때 발사한다.
                if (stateTimer.seconds() >= PRELOAD_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    beginFiring(AutoState.PRELOAD_FIRING);
                }
                break;

            case PRELOAD_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (isFiringComplete()) {
                    // 프리로드 발사 후 경로 1을 따라 아래 로딩존으로 이동하며 수집한다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    startPath(path1);
                    autoState = AutoState.DRIVE_PATH1_INTAKE;
                }
                break;

            case DRIVE_PATH1_INTAKE:
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (isPathFinishedOrTimedOut()) {
                    // 아래 로딩존 수집 후 경로 2로 슈팅 위치에 복귀한다.
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    startPath(path2);
                    autoState = AutoState.DRIVE_PATH2_SPINUP;
                }
                break;

            case DRIVE_PATH2_SPINUP:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isPathFinishedOrTimedOut()) {
                    beginShotWait(AutoState.WAIT_FOR_PATH2_SHOT);
                }
                break;

            case WAIT_FOR_PATH2_SHOT:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isReadyForPostArrivalShot()) {
                    beginFiring(AutoState.PATH2_FIRING);
                }
                break;

            case PATH2_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (isFiringComplete()) {
                    // 경로 3부터 반복 구간을 시작한다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    startPath(path3);
                    autoState = AutoState.DRIVE_PATH3_INTAKE;
                }
                break;

            case DRIVE_PATH3_INTAKE:
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (isPathFinishedOrTimedOut()) {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    startPath(path4);
                    autoState = AutoState.DRIVE_PATH4_SPINUP;
                }
                break;

            case DRIVE_PATH4_SPINUP:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isPathFinishedOrTimedOut()) {
                    beginShotWait(AutoState.WAIT_FOR_PATH4_SHOT);
                }
                break;

            case WAIT_FOR_PATH4_SHOT:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isReadyForPostArrivalShot()) {
                    beginFiring(AutoState.PATH4_FIRING);
                }
                break;

            case PATH4_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (isFiringComplete()) {
                    // 경로 5로 위쪽 로딩존을 수집하러 이동한다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    startPath(path5);
                    autoState = AutoState.DRIVE_PATH5_INTAKE;
                }
                break;

            case DRIVE_PATH5_INTAKE:
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                if (isPathFinishedOrTimedOut()) {
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    startPath(path6);
                    autoState = AutoState.DRIVE_PATH6_SPINUP;
                }
                break;

            case DRIVE_PATH6_SPINUP:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isPathFinishedOrTimedOut()) {
                    beginShotWait(AutoState.WAIT_FOR_PATH6_SHOT);
                }
                break;

            case WAIT_FOR_PATH6_SHOT:
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                if (isReadyForPostArrivalShot()) {
                    beginFiring(AutoState.PATH6_FIRING);
                }
                break;

            case PATH6_FIRING:
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                if (isFiringComplete()) {
                    // 경로 6 발사가 끝나면 경로 3부터 6까지 계속 반복한다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    startPath(path3);
                    autoState = AutoState.DRIVE_PATH3_INTAKE;
                }
                break;

            case PARKING:
            case DONE:
                // 주차 상태는 loop() 위쪽의 updateParking()에서 별도로 처리한다.
                break;
        }

        // 상태머신에서 지정한 명령을 실제 모터와 서보에 반영한다.
        shooter.update();
        artifactIntake.update();

        // 자율주행 종료 후 텔레옵이 이어받을 수 있도록 현재 Pose를 저장한다.
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        updateTelemetry();
    }

    @Override
    public void stop() {
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        follower.breakFollowing();
        shooter.stop();
        shooter.update();
        artifactIntake.setState(ArtifactIntake.State.IDLE);
        artifactIntake.update();
        turret.stop();
    }

    // 58초가 되는 순간 현재 위치에서 최종 주차 위치로 향하는 경로를 만든다.
    private void beginParking() {
        parkingStarted = true;

        // 진행 중인 수집/복귀 경로의 출력을 먼저 해제한다.
        follower.breakFollowing();
        shooter.stop();
        artifactIntake.setState(ArtifactIntake.State.IDLE);
        turret.stop();

        // 시간 초과로 예정 좌표에 도착하지 못했을 수 있으므로 실제 Pose를 시작점으로 사용한다.
        Pose currentPose = follower.getPose();
        Pose parkingStartPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                currentPose.getHeading());

        PathChain parkingPath = follower.pathBuilder()
                .addPath(new BezierLine(parkingStartPose, PARK_POSE))
                .setLinearHeadingInterpolation(
                        parkingStartPose.getHeading(), PARK_POSE.getHeading())
                .build();

        startPath(parkingPath);
        autoState = AutoState.PARKING;
    }

    // 주차 중에는 슈터, 인테이크, 터렛을 정지하고 주차 경로만 실행한다.
    private void updateParking() {
        shooter.stop();
        artifactIntake.setState(ArtifactIntake.State.IDLE);
        turret.stop();

        boolean parkFinished = !follower.isBusy();
        boolean autoTimeFinished = autoTimer.seconds() >= AUTO_END_SECONDS;

        // 주차에 도착했거나 60초가 되면 모든 주행 출력을 해제한다.
        if (parkFinished || autoTimeFinished) {
            follower.breakFollowing();
            autoState = AutoState.DONE;
        }

        shooter.update();
        artifactIntake.update();
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        updateTelemetry();

        // 주차 성공 시 즉시 종료하고, 실패하더라도 60초에는 반드시 종료한다.
        if (parkFinished || autoTimeFinished) {
            requestOpModeStop();
        }
    }

    // 경로 시작과 동시에 경로 제한시간 타이머를 초기화한다.
    private void startPath(PathChain path) {
        pathTimer.reset();
        follower.followPath(path);
    }

    // 정상 도착 또는 시간 초과가 발생하면 다음 상태로 진행하도록 true를 반환한다.
    private boolean isPathFinishedOrTimedOut() {
        if (!follower.isBusy()) {
            lastPathTimedOut = false;
            return true;
        }

        if (pathTimer.seconds() < PATH_TIMEOUT_SECONDS) {
            return false;
        }

        // 제한시간 이후에는 추종 출력을 해제해 벽을 계속 밀지 않게 한다.
        follower.breakFollowing();
        lastPathTimedOut = true;
        return true;
    }

    // 슈팅 위치에 도착한 순간 발사 전 대기시간 측정을 시작한다.
    private void beginShotWait(AutoState waitState) {
        stateTimer.reset();
        autoState = waitState;
    }

    // 도착 후 대기시간과 슈터 READY 조건이 모두 만족됐는지 확인한다.
    private boolean isReadyForPostArrivalShot() {
        return stateTimer.seconds() >= POST_ARRIVAL_SHOT_DELAY_SECONDS
                && shooter.getState() == Shooter.State.READY;
    }

    // 발사를 시작하고 아웃테이크 유지시간 측정을 시작한다.
    private void beginFiring(AutoState firingState) {
        shooter.fire();
        artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
        stateTimer.reset();
        autoState = firingState;
    }

    // 세 개의 유물을 밀어내는 발사시간이 끝났는지 확인한다.
    private boolean isFiringComplete() {
        return stateTimer.seconds() >= FIRING_TIME_SECONDS;
    }

    // 텔레옵과 같은 Pedro Pose/속도 및 ShotCalculator로 자동 조준한다.
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
        telemetry.addData("autoTime", autoTimer.seconds());
        telemetry.addData("parkingStarted", parkingStarted);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("headingDeg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("pathBusy", follower.isBusy());
        telemetry.addData("pathTime", pathTimer.seconds());
        telemetry.addData("pathTimedOut", lastPathTimedOut);
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
