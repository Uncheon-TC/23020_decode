package org.firstinspires.ftc.teamcode.subConstant;

public class ShooterConst {
    public enum Goal {
        RED,
        BLUE
    }

    public static double flywheel_P = 300;
    public static double flywheel_I = 0;
    public static double flywheel_D = 5;
    public static double flywheel_F = 0;

    public static double turret_P = 0.5;
    public static double turret_I = 0;
    public static double turret_D = 0.005;
    public static double turret_F = 0.02;
    public static double TURRET_DEADBAND_RADIANS = Math.toRadians(2);
    public static double TURRET_MAX_POWER = 0.7;
    public static double TURRET_INTEGRAL_LIMIT = 0.5;
    public static double TURRET_MAX_PID_DT_SECONDS = 0.1;
    public static double TURRET_MIN_ANGLE_RADIANS = Math.toRadians(-180);
    public static double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(180);

    public static String TURRET_LEFT_SERVO_NAME = "TL";
    public static String TURRET_RIGHT_SERVO_NAME = "TR";
    public static String TURRET_ENCODER_NAME = "Tencoder";

    // Servo rotations per one turret rotation.
    public static double TURRET_GEAR_RATIO = (30.0/60.0)*(105.0/25.0);
    public static double TURRET_ENCODER_DIRECTION = 1; //반시계방향으로 회전시 각도 증가
    public static double TURRET_LEFT_POWER_DIRECTION = -1;
    public static double TURRET_RIGHT_POWER_DIRECTION = -1;

    public static double RED_GOAL_X = 144;
    public static double RED_GOAL_Y = 144;
    public static double BLUE_GOAL_X = 0;
    public static double BLUE_GOAL_Y = 144;
    public static double SCORE_HEIGHT = 24.5;
    public static double SCORE_ANGLE = Math.toRadians(-35);

    public static double HOOD_MIN_ANGLE = Math.toRadians(41);
    public static double HOOD_MAX_ANGLE = Math.toRadians(55);
    public static double HOOD_SERVO_MIN = 0.79; // 41도 일때 값을 찾아넣어야함.
    public static double HOOD_SERVO_MAX = 0.26; // 55도 일때 값을 찾아넣어야함.

    public static double FLYWHEEL_TPR = 28; // 5000series 6000rpm motor
    public static double WHEEL_RADIUS = 1.417; // 라이노휠 큰거 1.89
    public static double SHOOTER_POWER_RATIO = 2.3; // 가까운 거리에서의 파워 조절
    public static double SHOOTER_POWER_RATIO_MAX = 3.0; // 먼거리에서의 파워 조절
    public static double TURRET_MOVEMENT_COMPENSATION = 1.2;
    public static double SHOOTER_BACKWARD_VELOCITY_COMPENSATION = 1.2; // 슈터 속도 보정(이동하면서 슈팅할때 속도 값을 25% 더 강하게 반영)

    // The ratio reaches its maximum at this field position.
    public static double SHOOTER_POWER_RATIO_FAR_X = 72;
    public static double SHOOTER_POWER_RATIO_FAR_Y = 0;
}
