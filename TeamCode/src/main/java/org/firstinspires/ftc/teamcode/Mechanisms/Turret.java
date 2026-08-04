package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

public class Turret {
    private CRServo leftServo;
    private CRServo rightServo;
    private AnalogInput encoder;

    // 아날로그 엔코더는 한 바퀴마다 전압이 반복되므로 회전량을 이어서 저장한다.
    // OpMode를 바꿔도 서보축의 누적 회전수를 잃지 않도록 공유한다.
    // 아날로그 엔코더는 한 바퀴 위치만 알려주므로 이 값이 사라지면
    // 같은 전압을 서로 다른 터렛 각도로 잘못 해석할 수 있다.
    private static boolean angleTrackingInitialized;
    private static double previousServoAngle;
    private static double continuousServoAngle;

    // PID 계산에 사용하는 값
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integral;
    private double previousError;

    private double targetAngleRadians;
    private double currentAngleRadians;
    private double servoPower;

    public void init(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(
                CRServo.class, ShooterConst.TURRET_LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(
                CRServo.class, ShooterConst.TURRET_RIGHT_SERVO_NAME);
        encoder = hardwareMap.get(
                AnalogInput.class, ShooterConst.TURRET_ENCODER_NAME);

        double servoAngle = readServoAngle();
        if (!angleTrackingInitialized) {
            // 새로 시작할 때는 케이블을 풀고 터렛을 정면에 놓아야 한다.
            // 단회전 엔코더만으로는 이전 전원의 누적 회전수를 복원할 수 없다.
            previousServoAngle = servoAngle;
            continuousServoAngle = 0;
            angleTrackingInitialized = true;
        } else {
            // Red/Blue OpMode 전환 중 발생한 작은 변화도 기존 누적각에 이어 붙인다.
            continuousServoAngle += wrapAngle(servoAngle - previousServoAngle);
            previousServoAngle = servoAngle;
        }
        updateCurrentAngle();

        resetPid();
        setServoPower(0);
    }

    public void trackPoint(Pose robotPose, double goalX, double goalY) {
        trackPoint(robotPose, goalX, goalY, 0);
    }

    public void trackPoint(Pose robotPose, double goalX, double goalY,
                           double shotOffsetRadians) {
        if (robotPose == null) {
            stop();
            return;
        }

        targetAngleRadians = calculateTargetAngle(
                robotPose, goalX, goalY, shotOffsetRadians);
        trackAngle(targetAngleRadians);
    }

    public void trackAngle(double angleRadians) {
        targetAngleRadians = Range.clip(
                wrapAngle(angleRadians),
                ShooterConst.TURRET_MIN_ANGLE_RADIANS,
                ShooterConst.TURRET_MAX_ANGLE_RADIANS);
        updateCurrentAngle();

        // 현재 누적각과 직접 비교해야 ±180도 경계를 넘어가지 않는다.
        // 목표가 +180도에서 -180도로 바뀌면 정면 쪽으로 반대로 돌아간다.
        double error = targetAngleRadians - currentAngleRadians;
        double power = calculatePidPower(error);

        // 제한각 밖으로 더 감기는 방향의 출력은 항상 차단한다.
        if ((currentAngleRadians >= ShooterConst.TURRET_MAX_ANGLE_RADIANS && power > 0)
                || (currentAngleRadians <= ShooterConst.TURRET_MIN_ANGLE_RADIANS && power < 0)) {
            power = 0;
        }
        setServoPower(power);
    }

    public void resetAngleTracking() {
        previousServoAngle = readServoAngle();
        continuousServoAngle = 0;
        updateCurrentAngle();
        resetPid();
    }

    public void stop() {
        resetPid();
        if (leftServo != null && rightServo != null) {
            setServoPower(0);
        }
    }

    // 필드의 골 방향을 로봇 기준 터렛 각도로 변환한다.
    private double calculateTargetAngle(Pose robotPose, double goalX, double goalY,
                                        double shotOffsetRadians) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();
        double fieldAngleToGoal = Math.atan2(goalY - robotY, goalX - robotX);

        return wrapAngle(fieldAngleToGoal - robotHeading - shotOffsetRadians);
    }

    // 엔코더의 한 바퀴 경계를 감지하여 서보의 누적 회전각을 계산한다.
    private void updateCurrentAngle() {
        double servoAngle = readServoAngle();
        double angleChange = wrapAngle(servoAngle - previousServoAngle);

        continuousServoAngle += angleChange;
        previousServoAngle = servoAngle;

        // 기어비: 서보 회전수 / 터렛 회전수
        currentAngleRadians = continuousServoAngle / ShooterConst.TURRET_GEAR_RATIO;
    }

    // 절대 영점 없이 현재 전압을 서보축의 한 바퀴 각도로 변환한다.
    // 실제 터렛 각도는 이전 측정값과의 변화량을 누적해서 계산한다.
    private double readServoAngle() {
        double angle = encoder.getVoltage()
                / encoder.getMaxVoltage() * 2.0 * Math.PI;

        return wrapAngle(angle * ShooterConst.TURRET_ENCODER_DIRECTION);
    }

    private double calculatePidPower(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        double derivative = 0;
        if (dt > 0 && dt <= ShooterConst.TURRET_MAX_PID_DT_SECONDS) {
            integral = Range.clip(
                    integral + error * dt,
                    -ShooterConst.TURRET_INTEGRAL_LIMIT,
                    ShooterConst.TURRET_INTEGRAL_LIMIT);
            derivative = (error - previousError) / dt;
        }
        previousError = error;

        // 목표 각도 근처에서는 떨림을 막기 위해 서보를 정지한다.
        if (Math.abs(error) <= ShooterConst.TURRET_DEADBAND_RADIANS) {
            integral = 0;
            return 0;
        }

        double power = ShooterConst.turret_P * error
                + ShooterConst.turret_I * integral
                + ShooterConst.turret_D * derivative
                + ShooterConst.turret_F * Math.signum(error);

        return Range.clip(
                power,
                -ShooterConst.TURRET_MAX_POWER,
                ShooterConst.TURRET_MAX_POWER);
    }

    // 두 서보가 같은 실제 방향으로 회전하도록 각각 방향 상수를 적용한다.
    private void setServoPower(double power) {
        servoPower = power;
        leftServo.setPower(power * ShooterConst.TURRET_LEFT_POWER_DIRECTION);
        rightServo.setPower(power * ShooterConst.TURRET_RIGHT_POWER_DIRECTION);
    }

    private void resetPid() {
        integral = 0;
        previousError = 0;
        pidTimer.reset();
    }

    private double wrapAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians <= -Math.PI) {
            radians += 2.0 * Math.PI;
        }
        return radians;
    }

    public double getTargetAngleDegrees() {
        return Math.toDegrees(targetAngleRadians);
    }

    public double getCurrentAngleDegrees() {
        return Math.toDegrees(currentAngleRadians);
    }

    public double getAngleErrorDegrees() {
        return Math.toDegrees(targetAngleRadians - currentAngleRadians);
    }

    public double getEncoderVoltage() {
        return encoder.getVoltage();
    }

    public double getServoPower() {
        return servoPower;
    }
}
