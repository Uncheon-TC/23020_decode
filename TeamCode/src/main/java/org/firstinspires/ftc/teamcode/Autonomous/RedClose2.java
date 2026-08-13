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

@Autonomous(name = "AutoRedClose2 30s/60s", group = "32020 AUTO")
public class RedClose2 extends OpMode {
    // 자율주행의 현재 진행 단계를 나타낸다.
    // loop()가 반복될 때 현재 상태에 해당하는 case만 실행된다.
    private enum AutoState {
        DRIVE_TO_FIRST_AND_SHOOT,           // 경로 1 이동 및 첫 번째 발사
        DRIVE_TO_SECOND_INTAKE,             // 경로 2 이동 및 첫 번째 줄 수집
        DRIVE_TO_THIRD_SPINUP,              // 경로 3 복귀 및 슈터 가속
        WAIT_FOR_SECOND_SHOT,               // 두 번째 발사 조건 확인
        SECOND_FIRING,                      // 두 번째 발사 진행
        DRIVE_FOURTH_AND_FIFTH_INTAKE,      // 경로 4~5 체인 이동 및 수집
        DRIVE_TO_SIXTH_SPINUP,              // 경로 6 이동 및 슈터 가속
        WAIT_FOR_THIRD_SHOT,                // 세 번째 발사 조건 확인
        THIRD_FIRING,                       // 세 번째 발사 진행
        DRIVE_SEVENTH_AND_EIGHTH_INTAKE,    // 경로 7~8 체인으로 게이트 수집
        WAIT_AT_GATE_INTAKE,                // 게이트에서 정지 상태로 추가 수집
        DRIVE_NINTH_AND_TENTH_SPINUP,       // 경로 9~10 체인 복귀 및 슈터 가속
        WAIT_FOR_GATE_SHOT,                 // 반복 발사 조건 확인
        GATE_FIRING,                        // 게이트에서 수집한 유물 발사
        DONE                                // 장치를 정지시키는 종료 상태
    }


    // 시작 후 너무 가까운 위치에서 발사하지 않도록 기다리는 시간이다.
    private static final double FIRST_SHOT_DELAY_SECONDS = 0.5;
    // 두 번째 발사부터 각 슈팅 위치에 도착한 뒤 기다리는 시간이다.
    private static final double POST_ARRIVAL_SHOT_DELAY_SECONDS = 0.2;
    // 한 번 발사할 때 아웃테이크를 유지하는 시간이다.
    private static final double FIRING_TIME_SECONDS = 0.5;

    // 경로 8 끝의 게이트 위치에서 추가로 수집하는 시간이다.
    private static final double INTAKING_TIME_SECONDS = 1.5;
    // 경로 또는 패스체인이 이 시간을 넘기면 강제로 종료하고 다음 상태로 진행한다.
    private static final double PATH_TIMEOUT_SECONDS = 2.5;
    // 일반 경로에서 사용하는 최대 구동 출력이다.
    private static final double DEFAULT_PATH_MAX_POWER = 1.0;
    // SIXTH_POSE에서 GATE_POSE로 진입할 때만 사용하는 낮은 최대 출력이다.
    private static final double GATE_APPROACH_MAX_POWER = 0.4;

    // 유물을 흡입하거나 슈터 방향으로 내보내는 인테이크 장치이다.
    private final ArtifactIntake artifactIntake = new ArtifactIntake();

    // 기존 자동 상수 객체이다. 현재 이 클래스에서는 값을 직접 읽지 않는다.
    private final AutoConst autoConst = new AutoConst();

    // Pose의 X, Y 단위는 인치이고 헤딩 단위는 라디안이다.
    // 자율주행 시작 위치: (109, 134), 로봇 방향 90도.
    private static final Pose START_POSE =
            new Pose(110, 133, Math.toRadians(90));

    // 첫 번째 슈팅 위치이며 게이트 반복 사이클의 슈팅 위치와는 구분된다.
    private static final Pose FIRST_POSE =
            new Pose(98, 84, Math.toRadians(0));

    // 첫 번째 줄의 유물을 먹기 위해 이동하는 위치이다.
    private static final Pose SECOND_POSE =
            new Pose(121, 84, Math.toRadians(0));

    // 경로 4의 베지어 곡선 모양을 결정하는 제어점이다.
    // 제어점은 곡선을 당기는 점이므로 로봇이 반드시 통과하지는 않는다.
    private static final Pose FOURTH_CONTROL_POSE =
            new Pose(92, 56.012, Math.toRadians(0));

    // 경로 4의 끝이자 경로 5의 시작 위치이다.
    private static final Pose THIRD_POSE =
            new Pose(121, 60, Math.toRadians(0));

    // 두 번째 줄 수집 후 게이트를 살짝 여는 위치이다.
    private static final Pose FOURTH_POSE =
            new Pose(125, 68, Math.toRadians(0));

    // 세 번째 슈팅 및 반복 슈팅에 사용하는 위치이다.
    private static final Pose FIFTH_POSE =
            new Pose(94, 79, Math.toRadians(340));

    // 게이트 진입 경로와 복귀 경로가 공통으로 지나는 중간 위치이다.
    private static final Pose SIXTH_POSE =
            new Pose(119, 69, Math.toRadians(340));

    // 게이트 안쪽에서 유물을 수집하는 최종 위치이다.
    private static final Pose GATE_POSE =
            new Pose(131.5, 62  , Math.toRadians(27));




    // 플라이휠 속도, 후드 각도, 발사 상태를 관리한다.
    private final Shooter shooter = new Shooter();
    // 현재 로봇 Pose를 이용해 레드 골대를 계속 조준한다.
    private final Turret turret = new Turret();
    // 현재 상태 안에서 발사 또는 수집 시간을 측정한다.
    private final ElapsedTime stateTimer = new ElapsedTime();
    // 발사/수집 시간과 별개로 현재 경로의 실행시간만 측정한다.
    private final ElapsedTime pathTimer = new ElapsedTime();

    // Pedro Pathing의 위치 추정, 속도 계산, 경로 추종을 담당한다.
    private Follower follower;
    // 경로 1: 시작 위치에서 첫 번째 슈팅 위치로 이동한다.
    private PathChain firstPath;
    // 경로 2: 첫 번째 줄의 유물을 먹으러 이동한다.
    private PathChain secondPath;
    // 경로 3: 두 번째 슈팅을 위해 FIRST_POSE로 복귀한다.
    private PathChain thirdPath;
    // 경로 4와 5를 연결한 체인이다.
    private PathChain fourthAndFifthPath;
    // 경로 6: 세 번째 슈팅 위치로 이동한다.
    private PathChain sixthPath;
    // 경로 7과 8을 연결한 게이트 진입 체인이다.
    private PathChain seventhAndEighthPath;
    // 경로 9와 10을 연결한 슈팅 위치 복귀 체인이다.
    private PathChain ninthAndTenthPath;
    // 시작 시 가장 먼저 경로 1 이동 및 발사 상태를 실행한다.
    private AutoState autoState = AutoState.DRIVE_TO_FIRST_AND_SHOOT;
    // 거리 및 로봇 속도로 계산된 슈터/터렛 보정 결과를 저장한다.
    private ShotCalculator.ShotResult shotResult;
    // 첫 발사 명령이 이미 시작됐는지 기록한다.
    private boolean firstShotStarted;
    // 첫 발사의 아웃테이크 유지 시간이 끝났는지 기록한다.
    private boolean firstShotComplete;
    // 가장 최근 경로가 정상 완료가 아닌 시간 초과로 끝났는지 기록한다.
    private boolean lastPathTimedOut;

    @Override
    public void init() {
        // Constants에 설정된 Pinpoint 로컬라이저와 구동 상수로 Follower를 생성한다.
        follower = Constants.createFollower(hardwareMap);
        // Follower의 최초 위치를 START_POSE로 지정한다.
        follower.setStartingPose(START_POSE);

        // 하드웨어맵에서 인테이크 모터와 서보를 연결한다.
        artifactIntake.init(hardwareMap);
        // 하드웨어맵에서 슈터 모터와 후드 서보를 연결한다.
        shooter.init(hardwareMap);
        // 하드웨어맵에서 터렛 CR 서보와 아날로그 엔코더를 연결한다.
        turret.init(hardwareMap);

        // 경로 1: 시작 위치에서 첫 번째 슈팅 위치까지 직선으로 이동한다.
        firstPath = follower.pathBuilder()
                // START_POSE와 FIRST_POSE를 잇는 직선 경로를 추가한다.
                .addPath(new BezierLine(START_POSE, FIRST_POSE))
                // 이동하는 동안 헤딩을 90도에서 0도로 연속적으로 변경한다.
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(), FIRST_POSE.getHeading())
                // 설정한 경로를 실행 가능한 PathChain으로 생성한다.
                .build();

        // 경로 2: 첫 번째 슈팅 위치에서 첫 번째 줄 수집 위치로 이동한다.
        secondPath = follower.pathBuilder()
                .addPath(new BezierLine(FIRST_POSE, SECOND_POSE))
                // 이동하는 동안 로봇 헤딩을 0도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 3: 수집 위치에서 두 번째 슈팅을 위해 되돌아온다.
        thirdPath = follower.pathBuilder()
                .addPath(new BezierLine(SECOND_POSE, FIRST_POSE))
                // 복귀하는 동안에도 로봇 헤딩을 0도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 4와 5: 두 번째 줄을 수집한 뒤 게이트를 살짝 연다.
        fourthAndFifthPath = follower.pathBuilder()
                // 경로 4는 제어점을 이용한 곡선으로 두 번째 줄에 접근한다.
                .addPath(new BezierCurve(FIRST_POSE, FOURTH_CONTROL_POSE, THIRD_POSE))
                // 경로 4를 이동하는 동안 헤딩은 0도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // 경로 5를 같은 체인에 추가하여 중간 정지 없이 이어서 이동한다.
                .addPath(new BezierLine(THIRD_POSE, FOURTH_POSE))
                // 경로 5에서도 헤딩은 0도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 경로 6: 게이트를 살짝 연 위치에서 세 번째 슈팅 위치로 이동한다.
        sixthPath = follower.pathBuilder()
                .addPath(new BezierLine(FOURTH_POSE, FIFTH_POSE))
                // 로봇 방향을 340도, 즉 -20도와 같은 방향으로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(340))
                .build();

        // 경로 7과 8: 게이트 안쪽으로 들어가며 유물을 수집한다.
        seventhAndEighthPath = follower.pathBuilder()
                // 경로 7: 슈팅 위치에서 게이트 앞의 중간 위치로 이동한다.
                .addPath(new BezierLine(FIFTH_POSE, SIXTH_POSE))
                // 경로 7에서는 헤딩을 340도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(340))
                // 경로 8: 중간 위치에서 게이트 안쪽 최종 위치로 이동한다.
                .addPath(new BezierLine(SIXTH_POSE, GATE_POSE))
                // 게이트에 진입하면서 헤딩을 340도에서 27도로 변경한다.
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(27))
                // 경로 8이 시작되는 순간 최대 출력을 낮춰 게이트에 천천히 진입한다.
                .addParametricCallback(
                        0.0,
                        () -> follower.setMaxPowerScaling(GATE_APPROACH_MAX_POWER))
                .build();

        // 경로 9와 10: 게이트에서 슈팅 위치로 복귀한다.
        ninthAndTenthPath = follower.pathBuilder()
                // 경로 9: 게이트 안쪽에서 중간 위치로 빠져나온다.
                .addPath(new BezierLine(GATE_POSE, SIXTH_POSE))
                // 빠져나오면서 헤딩을 27도에서 340도로 되돌린다.
                .setLinearHeadingInterpolation(Math.toRadians(27), Math.toRadians(340))
                // 경로 10: 중간 위치에서 반복 슈팅 위치로 복귀한다.
                .addPath(new BezierLine(SIXTH_POSE, FIFTH_POSE))
                // 경로 10에서는 헤딩을 340도로 유지한다.
                .setConstantHeadingInterpolation(Math.toRadians(340))
                .build();




        // 이전 자율주행에서 저장됐을 수 있는 Pose를 제거한다.
        // 이번 자율주행의 최종 Pose는 loop()와 stop()에서 다시 저장된다.
        PoseStorage.clearAutoPose();
    }

    @Override
    public void start() {
        // START 버튼을 누른 순간 로컬라이저 Pose를 시작 좌표로 확정한다.
        follower.setPose(START_POSE);
        // 경로 1을 이동하면서 발사할 수 있도록 즉시 플라이휠 가속을 시작한다.
        shooter.spinUp();
        // 이전 실행의 첫 발사 상태가 남지 않도록 초기화한다.
        firstShotStarted = false;
        // 첫 발사가 아직 완료되지 않은 상태로 초기화한다.
        firstShotComplete = false;
        // 첫 발사 지연시간을 START 버튼을 누른 시점부터 측정한다.
        stateTimer.reset();
        // 경로 1 추종을 시작한다.
        startPath(firstPath);
        // 상태머신도 경로 1 이동 및 발사 단계에서 시작한다.
        autoState = AutoState.DRIVE_TO_FIRST_AND_SHOOT;
    }

    @Override
    public void loop() {
        // Pinpoint Pose를 갱신하고 현재 경로에 필요한 메카넘 휠 출력을 계산한다.
        follower.update();
        // 갱신된 Pose와 속도로 슈터 및 터렛 목표값을 매 주기 다시 계산한다.
        updateAutomaticAim();

        // 현재 autoState에 해당하는 동작만 실행한다.
        switch (autoState) {
            case DRIVE_TO_FIRST_AND_SHOOT:
                // 경로 1 이동 중에도 플라이휠 목표속도를 계속 유지한다.
                shooter.spinUp();
                // 첫 발사와 별개로 경로 완료 또는 3초 초과를 매 loop 확인한다.
                boolean firstPathFinished = isPathFinishedOrTimedOut();

                // 첫 발사를 아직 시작하지 않았고,
                // START 후 0.5초가 지났으며 슈터가 목표속도에 도달해야 발사한다.
                if (!firstShotStarted
                        && stateTimer.seconds() >= FIRST_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    // Shooter 상태를 FIRING으로 전환한다.
                    shooter.fire();
                    // 지금부터 첫 발사의 아웃테이크 시간을 새로 측정한다.
                    stateTimer.reset();
                    // 같은 발사 명령이 다음 loop에서 다시 실행되지 않게 기록한다.
                    firstShotStarted = true;
                }

                // 첫 발사를 시작했고 발사 시간이 끝나지 않았으면 유물을 밀어낸다.
                if (firstShotStarted && !firstShotComplete) {
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    // 정해진 발사 시간이 지나면 첫 발사가 완료된 것으로 처리한다.
                    if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                        firstShotComplete = true;
                        // 발사 완료 후 인테이크 장치를 정지한다.
                        artifactIntake.setState(ArtifactIntake.State.IDLE);
                    }
                } else {
                    // 발사 전이거나 발사가 끝난 동안에는 인테이크를 정지한다.
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                }

                // 정상 상황에서는 첫 발사와 경로 1이 모두 끝난 뒤 넘어간다.
                // 경로가 3초를 초과한 경우에는 첫 발사 완료 여부와 관계없이 다음 경로로 넘어간다.
                if ((firstShotComplete && firstPathFinished) || lastPathTimedOut) {
                    // 첫 번째 줄을 수집하는 동안에는 슈터를 정지한다.
                    shooter.stop();
                    // 경로 2를 이동하기 전에 인테이크를 수집 방향으로 켠다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    // 첫 번째 줄 수집 경로를 시작한다.
                    startPath(secondPath);
                    // 다음 loop부터 경로 2 수집 상태를 실행한다.
                    autoState = AutoState.DRIVE_TO_SECOND_INTAKE;
                }
                break;

            case DRIVE_TO_SECOND_INTAKE:
                // 수집 중 불필요한 전력 소비를 줄이기 위해 슈터를 정지한다.
                shooter.stop();
                // 경로 2 전체에서 인테이크를 계속 작동시킨다.
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                // 경로 2가 끝나 SECOND_POSE에 도착했는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 수집 경로가 끝났으므로 인테이크를 잠시 정지한다.
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    // 경로 3 복귀 중 목표속도에 도달하도록 슈터를 미리 가속한다.
                    shooter.spinUp();
                    // 두 번째 슈팅 위치로 돌아가는 경로 3을 시작한다.
                    startPath(thirdPath);
                    // 다음 loop부터 경로 3 복귀 상태를 실행한다.
                    autoState = AutoState.DRIVE_TO_THIRD_SPINUP;
                }
                break;

            case DRIVE_TO_THIRD_SPINUP:
                // 경로 3 이동 중 플라이휠을 계속 가속한다.
                shooter.spinUp();
                // 발사 위치까지 이동하는 동안 인테이크는 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 경로 3이 끝나 FIRST_POSE에 도착했는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 도착 후 발사 지연시간을 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 슈터 준비와 지연시간을 확인하는 상태로 전환한다.
                    autoState = AutoState.WAIT_FOR_SECOND_SHOT;
                }
                break;

            case WAIT_FOR_SECOND_SHOT:
                // 대기 중에도 슈터 목표속도를 유지한다.
                shooter.spinUp();
                // 발사 조건을 기다리는 동안 인테이크는 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 도착 후 0.5초가 지났고 슈터가 READY일 때만 두 번째 발사를 시작한다.
                if (stateTimer.seconds() >= POST_ARRIVAL_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    // Shooter 상태를 FIRING으로 전환한다.
                    shooter.fire();
                    // 유물을 플라이휠 쪽으로 밀어낸다.
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    // 두 번째 발사 시간을 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 다음 loop부터 두 번째 발사 유지 상태를 실행한다.
                    autoState = AutoState.SECOND_FIRING;
                }
                break;

            case SECOND_FIRING:
                // 두 번째 발사 시간 동안 아웃테이크를 계속 유지한다.
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                // 발사 유지시간 0.5초가 끝났는지 확인한다.
                if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    // 다음 수집 구간에서는 슈터가 필요하지 않으므로 정지한다.
                    shooter.stop();
                    // 경로 4~5를 시작하기 전에 인테이크를 켠다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    // 두 번째 줄 수집과 게이트 열기를 하나의 패스체인으로 시작한다.
                    startPath(fourthAndFifthPath);
                    // 다음 loop부터 경로 4~5 수집 상태를 실행한다.
                    autoState = AutoState.DRIVE_FOURTH_AND_FIFTH_INTAKE;
                }
                break;

            case DRIVE_FOURTH_AND_FIFTH_INTAKE:
                // 경로 4~5를 이동하는 동안 슈터는 정지한다.
                shooter.stop();
                // 두 번째 줄의 유물을 먹도록 체인 전체에서 인테이크를 유지한다.
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                // 경로 4와 경로 5가 모두 끝났는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 수집과 게이트 열기가 끝났으므로 인테이크를 정지한다.
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    // 경로 6 이동 중 세 번째 발사를 준비하도록 슈터를 가속한다.
                    shooter.spinUp();
                    // 세 번째 슈팅 위치로 가는 경로 6을 시작한다.
                    startPath(sixthPath);
                    // 다음 loop부터 경로 6 이동 상태를 실행한다.
                    autoState = AutoState.DRIVE_TO_SIXTH_SPINUP;
                }
                break;

            case DRIVE_TO_SIXTH_SPINUP:
                // 경로 6을 이동하면서 플라이휠 목표속도를 계속 유지한다.
                shooter.spinUp();
                // 슈팅 위치로 이동하는 동안 인테이크는 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 경로 6이 끝나 FIFTH_POSE에 도착했는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 세 번째 발사 전 0.5초 대기를 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 위치 도착 후 대기시간과 슈터 READY를 검사하는 상태로 전환한다.
                    autoState = AutoState.WAIT_FOR_THIRD_SHOT;
                }
                break;

            case WAIT_FOR_THIRD_SHOT:
                // 대기 중에도 플라이휠이 목표속도를 유지하도록 한다.
                shooter.spinUp();
                // 발사 전까지 인테이크를 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 도착 후 0.5초가 지났고 슈터가 목표속도에 도달했을 때만 발사한다.
                if (stateTimer.seconds() >= POST_ARRIVAL_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    // 세 번째 발사를 시작한다.
                    shooter.fire();
                    // 유물을 슈터 방향으로 밀어낸다.
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    // 세 번째 발사 유지시간을 측정한다.
                    stateTimer.reset();
                    // 다음 loop부터 세 번째 발사 상태를 실행한다.
                    autoState = AutoState.THIRD_FIRING;
                }
                break;

            case THIRD_FIRING:
                // 세 번째 발사 동안 아웃테이크를 계속 유지한다.
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                // 발사 유지시간이 끝났는지 확인한다.
                if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    // 게이트 수집 구간에서는 슈터를 정지한다.
                    shooter.stop();
                    // 게이트로 출발하기 전에 인테이크를 수집 방향으로 켠다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    // 경로 7과 8로 구성된 게이트 진입 체인을 시작한다.
                    startPath(seventhAndEighthPath);
                    // 다음 loop부터 게이트 이동 및 수집 상태를 실행한다.
                    autoState = AutoState.DRIVE_SEVENTH_AND_EIGHTH_INTAKE;
                }
                break;

            case DRIVE_SEVENTH_AND_EIGHTH_INTAKE:
                // 게이트로 이동하는 동안 슈터는 정지한다.
                shooter.stop();
                // 경로 7~8 전체에서 유물을 계속 수집한다.
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                // 두 경로가 모두 끝나 GATE_POSE에 도착했는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 게이트에서 추가로 수집할 2초를 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 로봇을 정지시킨 채 추가 수집하는 상태로 전환한다.
                    autoState = AutoState.WAIT_AT_GATE_INTAKE;
                }
                break;

            case WAIT_AT_GATE_INTAKE:
                // 게이트에서 기다리는 동안 슈터는 정지 상태를 유지한다.
                shooter.stop();
                // 로봇이 멈춘 상태에서도 인테이크를 돌려 남은 유물을 수집한다.
                artifactIntake.setState(ArtifactIntake.State.INTAKING);
                // 게이트 추가 수집시간 2초가 지났는지 확인한다.
                if (stateTimer.seconds() >= INTAKING_TIME_SECONDS) {
                    // 복귀 경로에서 플라이휠이 가속되도록 슈터를 먼저 켠다.
                    shooter.spinUp();
                    // 수집이 끝났으므로 인테이크를 정지한다.
                    artifactIntake.setState(ArtifactIntake.State.IDLE);
                    // 경로 9와 10으로 구성된 슈팅 위치 복귀 체인을 시작한다.
                    startPath(ninthAndTenthPath);
                    // 다음 loop부터 복귀 및 슈터 가속 상태를 실행한다.
                    autoState = AutoState.DRIVE_NINTH_AND_TENTH_SPINUP;
                }
                break;

            case DRIVE_NINTH_AND_TENTH_SPINUP:
                // 경로 9~10으로 복귀하는 동안 플라이휠 목표속도를 유지한다.
                shooter.spinUp();
                // 복귀 중에는 인테이크를 정지해 유물이 불필요하게 움직이지 않게 한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 두 경로가 모두 끝나 FIFTH_POSE에 도착했는지 확인한다.
                if (isPathFinishedOrTimedOut()) {
                    // 반복 발사 전 0.5초 대기를 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 도착 후 대기시간과 슈터 준비 여부를 확인하는 상태로 전환한다.
                    autoState = AutoState.WAIT_FOR_GATE_SHOT;
                }
                break;

            case WAIT_FOR_GATE_SHOT:
                // 대기 중에도 플라이휠 목표속도를 계속 유지한다.
                shooter.spinUp();
                // 발사 준비가 끝날 때까지 인테이크는 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                // 도착 후 0.5초가 지났고 슈터 속도가 목표값에 도달했을 때만 발사한다.
                if (stateTimer.seconds() >= POST_ARRIVAL_SHOT_DELAY_SECONDS
                        && shooter.getState() == Shooter.State.READY) {
                    // 게이트에서 수집한 유물의 발사를 시작한다.
                    shooter.fire();
                    // 유물을 플라이휠 쪽으로 밀어낸다.
                    artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                    // 반복 발사 유지시간을 측정하기 위해 타이머를 초기화한다.
                    stateTimer.reset();
                    // 다음 loop부터 반복 발사 상태를 실행한다.
                    autoState = AutoState.GATE_FIRING;
                }
                break;

            case GATE_FIRING:
                // 정해진 발사시간 동안 아웃테이크를 계속 유지한다.
                artifactIntake.setState(ArtifactIntake.State.OUTTAKING);
                // 발사시간 0.5초가 끝나면 다음 게이트 수집 사이클을 시작한다.
                if (stateTimer.seconds() >= FIRING_TIME_SECONDS) {
                    // 게이트 수집 중에는 슈터가 필요하지 않으므로 정지한다.
                    shooter.stop();
                    // 다음 게이트 이동을 시작하기 전에 인테이크를 켠다.
                    artifactIntake.setState(ArtifactIntake.State.INTAKING);
                    // 경로 7~8을 다시 시작하여 반복 사이클로 돌아간다.
                    startPath(seventhAndEighthPath);
                    // 이후 경로 7→8→9→10→발사를 OpMode 종료까지 반복한다.
                    autoState = AutoState.DRIVE_SEVENTH_AND_EIGHTH_INTAKE;
                }
                break;


            case DONE:
            default:
                // 종료 상태에서는 슈터 출력을 끈다.
                shooter.stop();
                // 종료 상태에서는 인테이크도 정지한다.
                artifactIntake.setState(ArtifactIntake.State.IDLE);
                break;

        }

        // 위 상태에서 지정한 Shooter 상태를 실제 모터/서보 출력에 반영한다.
        shooter.update();
        // 위 상태에서 지정한 인테이크 상태를 실제 장치 출력에 반영한다.
        artifactIntake.update();
        // 자율주행 종료 후 텔레옵이 이어받을 수 있도록 최신 Pose를 저장한다.
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        // 드라이버스테이션과 대시보드에 디버깅 값을 출력한다.
        updateTelemetry();
    }

    @Override
    public void stop() {
        // OpMode가 종료되는 순간의 최종 Pose를 한 번 더 저장한다.
        PoseStorage.saveAutoPose(follower.getPose(), ShooterConst.Goal.RED);
        // 슈터 상태를 정지 상태로 바꾼다.
        shooter.stop();
        // 정지 상태를 실제 슈터 모터와 서보에 즉시 반영한다.
        shooter.update();
        // 인테이크 상태를 정지 상태로 바꾼다.
        artifactIntake.setState(ArtifactIntake.State.IDLE);
        // 정지 상태를 실제 인테이크 장치에 즉시 반영한다.
        artifactIntake.update();
        // 터렛 CR 서보 출력을 0으로 만들어 회전을 멈춘다.
        turret.stop();
    }

    // 모든 경로 시작을 이 메서드로 통일하여 경로 전용 타이머가 빠짐없이 초기화되게 한다.
    private void startPath(PathChain path) {
        // 게이트 진입에서 낮춘 출력을 다음 경로 시작 전에 기본값으로 복원한다.
        follower.setMaxPowerScaling(DEFAULT_PATH_MAX_POWER);
        // 현재 경로의 2.5초 제한시간 측정을 시작한다.
        pathTimer.reset();
        // Pedro Follower에 새 경로 추종을 요청한다.
        follower.followPath(path);
    }

    // 경로가 정상 완료됐거나 2.5초 제한시간을 넘겼으면 true를 반환한다.
    private boolean isPathFinishedOrTimedOut() {
        // Follower가 더 이상 경로를 추종하지 않으면 정상 완료이다.
        if (!follower.isBusy()) {
            // 가장 최근에 끝난 경로가 정상 완료됐음을 기록한다.
            lastPathTimedOut = false;
            return true;
        }

        // 아직 2.5초가 지나지 않았으면 현재 경로를 계속 추종한다.
        if (pathTimer.seconds() < PATH_TIMEOUT_SECONDS) {
            return false;
        }

        // 2.5초를 넘긴 경로 출력을 해제하여 벽을 계속 밀지 않도록 한다.
        follower.breakFollowing();
        // 텔레메트리에서 시간 초과를 확인할 수 있도록 기록한다.
        lastPathTimedOut = true;
        // 상태머신이 기존 다음 상태로 진행하도록 true를 반환한다.
        return true;
    }

    // 현재 Pose와 이동 속도를 사용하여 슈터와 터렛의 목표값을 계산한다.
    private void updateAutomaticAim() {
        // Pinpoint를 사용하는 Follower에서 현재 필드 Pose를 읽는다.
        Pose pose = follower.getPose();
        // Follower에서 현재 필드 기준 X/Y 속도 벡터를 읽는다.
        Vector velocity = follower.getVelocity();

        // 현재 위치에서 레드 골대까지의 거리와 로봇 이동 속도를 반영한다.
        // 반환값에는 플라이휠 속도, 후드 각도, 이동 중 터렛 보정값이 들어 있다.
        shotResult = shooter.aimAt(
                pose,
                ShooterConst.RED_GOAL_X,
                ShooterConst.RED_GOAL_Y,
                velocity.getXComponent(),
                velocity.getYComponent());

        // 슈팅 계산이 성공했으면 이동 속도로 계산된 터렛 리드 보정값도 적용한다.
        if (shotResult != null) {
            turret.trackPoint(
                    pose,
                    ShooterConst.RED_GOAL_X,
                    ShooterConst.RED_GOAL_Y,
                    shotResult.turretOffset);
        } else {
            // 계산 결과가 없으면 속도 보정 없이 골대 좌표만 조준한다.
            turret.trackPoint(
                    pose,
                    ShooterConst.RED_GOAL_X,
                    ShooterConst.RED_GOAL_Y);
        }
    }

    // 자율주행 상태와 장치 값을 드라이버스테이션 텔레메트리에 표시한다.
    private void updateTelemetry() {
        // 같은 loop에서 사용할 현재 Pose를 한 번만 읽는다.
        Pose pose = follower.getPose();
        // 현재 상태머신 단계를 표시한다.
        telemetry.addData("autoState", autoState);
        // Pinpoint/Pedro가 추정한 필드 X 좌표를 표시한다.
        telemetry.addData("x", pose.getX());
        // Pinpoint/Pedro가 추정한 필드 Y 좌표를 표시한다.
        telemetry.addData("y", pose.getY());
        // 라디안 헤딩을 사람이 읽기 쉬운 각도로 바꾸어 표시한다.
        telemetry.addData("headingDeg", Math.toDegrees(pose.getHeading()));
        // Follower가 아직 경로를 추종 중인지 표시한다.
        telemetry.addData("pathBusy", follower.isBusy());
        // 현재 경로가 시작된 뒤 흐른 시간을 표시한다.
        telemetry.addData("pathTime", pathTimer.seconds());
        // 가장 최근 경로가 2.5초 시간 초과로 종료됐는지 표시한다.
        telemetry.addData("pathTimedOut", lastPathTimedOut);
        // 현재 슈터 상태를 표시한다.
        telemetry.addData("shooterState", shooter.getState());
        // 왼쪽 슈터 모터 엔코더가 측정한 실제 속도를 표시한다.
        telemetry.addData("shooterVelocity", shooter.ShooterLeft.getVelocity());
        // 거리와 움직임 보정으로 계산된 목표 슈터 속도를 표시한다.
        telemetry.addData("shooterTarget", shooter.getTargetVelocity());
        // 현재 거리에 적용된 슈터 파워 비율을 표시한다.
        telemetry.addData("shooterPowerRatio", shooter.getAppliedPowerRatio());
        // 현재 인테이크 상태를 표시한다.
        telemetry.addData("intakeState", artifactIntake.getState());
        // 터렛이 이동하려는 목표 상대각도를 표시한다.
        telemetry.addData("turretTargetDeg", turret.getTargetAngleDegrees());
        // 아날로그 엔코더로 측정한 터렛의 현재 상대각도를 표시한다.
        telemetry.addData("turretCurrentDeg", turret.getCurrentAngleDegrees());
        // 두 터렛 CR 서보에 적용되는 제어 출력을 표시한다.
        telemetry.addData("turretPower", turret.getServoPower());
        // 슈팅 계산 결과가 있을 때만 거리와 후드 각도를 표시한다.
        if (shotResult != null) {
            // 현재 로봇 위치에서 레드 골대까지 계산된 거리를 표시한다.
            telemetry.addData("shotDistance", shotResult.distanceToGoal);
            // 계산된 후드 각도를 라디안에서 도 단위로 바꾸어 표시한다.
            telemetry.addData("hoodAngleDeg", Math.toDegrees(shotResult.hoodAngle));
        }
        // 이번 loop에서 추가한 모든 텔레메트리 값을 전송한다.
        telemetry.update();
    }
}
