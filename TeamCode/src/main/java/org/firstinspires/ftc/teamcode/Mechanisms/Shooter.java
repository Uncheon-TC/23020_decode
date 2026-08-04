package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;


public class Shooter {
    public enum State {
        IDLE,
        SPINNING_UP,
        READY,
        FIRING
    }

    private static final double TARGET_VELOCITY = 2000;
    private static final double VELOCITY_TOLERANCE = 100;
    private static final double FIRE_TIME_SECONDS = 0.35;

    public DcMotorEx ShooterLeft, ShooterRight;
    private Servo HoodServo;
    private State state = State.IDLE;
    private ElapsedTime fireTimer = new ElapsedTime();
    private double targetVelocity = TARGET_VELOCITY;
    private ShotCalculator.ShotResult lastShotResult;

    public void init(HardwareMap hwMap){
        ShooterLeft = hwMap.get(DcMotorEx.class, "SL");
        ShooterRight = hwMap.get(DcMotorEx.class, "SR");
        HoodServo = hwMap.get(Servo.class, "BS");

        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterLeft.setDirection(DcMotor.Direction.FORWARD);
        ShooterRight.setDirection(DcMotor.Direction.REVERSE);

        ShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        PIDFCoefficients flywheelPidfCoefficients = new PIDFCoefficients(
                org.firstinspires.ftc.teamcode.subConstant.ShooterConst.flywheel_P,
                org.firstinspires.ftc.teamcode.subConstant.ShooterConst.flywheel_I,
                org.firstinspires.ftc.teamcode.subConstant.ShooterConst.flywheel_D,
                org.firstinspires.ftc.teamcode.subConstant.ShooterConst.flywheel_F);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidfCoefficients);
        HoodServo.setPosition(ShooterConst.HOOD_SERVO_MIN);

    }

    public void spinUp() {
        if (state == State.IDLE) {
            state = State.SPINNING_UP;
        }
    }

    public void fire() {
        if (state == State.READY) {
            state = State.FIRING;
            fireTimer.reset();
        }
    }

    public void stop() {
        state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    public ShotCalculator.ShotResult aimAt(Pose robotPose, double goalX, double goalY) {
        return aimAt(robotPose, goalX, goalY, 0, 0);
    }

    public ShotCalculator.ShotResult aimAt(Pose robotPose, double goalX, double goalY,
                                           double robotVelocityX, double robotVelocityY) {
        if (robotPose == null) {
            lastShotResult = null;
            targetVelocity = TARGET_VELOCITY;
            return null;
        }

        ShotCalculator.ShotResult result = ShotCalculator.calculateShot(
                robotPose.getX(),
                robotPose.getY(),
                goalX,
                goalY,
                ShooterConst.SCORE_HEIGHT,
                robotVelocityX,
                robotVelocityY,
                ShooterConst.SCORE_ANGLE);

        if (result == null) {
            lastShotResult = null;
            targetVelocity = TARGET_VELOCITY;
            return null;
        }

        double hoodAngle = Range.clip(
                result.hoodAngle,
                ShooterConst.HOOD_MIN_ANGLE,
                ShooterConst.HOOD_MAX_ANGLE);
        HoodServo.setPosition(mapAngleToServo(hoodAngle));
        targetVelocity = velocityToTicks(result.launchSpeed) * ShooterConst.SHOOTER_POWER_RATIO;
        lastShotResult = result;
        return result;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public ShotCalculator.ShotResult getLastShotResult() {
        return lastShotResult;
    }

    public void update() {
        switch (state) {
            case SPINNING_UP:
                setVelocity(targetVelocity);
                if (isReadyToFire()) {
                    state = State.READY;
                }
                break;
            case READY:
                setVelocity(targetVelocity);
                if (!isReadyToFire()) {
                    state = State.SPINNING_UP;
                }
                break;
            case FIRING:
                setVelocity(targetVelocity);
                if (fireTimer.seconds() >= FIRE_TIME_SECONDS) {
                    state = State.READY;
                }
                break;
            case IDLE:
            default:
                setVelocity(0);
                break;
        }
    }

    private boolean isReadyToFire() {
        return Math.abs(targetVelocity - ShooterLeft.getVelocity()) <= VELOCITY_TOLERANCE;
    }

    private void setVelocity(double shooterVelocity){
        if (shooterVelocity <= 0) {
            ShooterLeft.setVelocity(0);
            ShooterRight.setPower(0);
            return;
        }

        ShooterLeft.setVelocity(shooterVelocity);
        // SR에는 엔코더가 없으므로 SL의 PIDF 출력 파워를 그대로 따라간다.
        ShooterRight.setPower(ShooterLeft.getPower());
    }

    private double velocityToTicks(double velocityInchesPerSecond) {
        double wheelCircumference = 2 * Math.PI * ShooterConst.WHEEL_RADIUS;
        double wheelRevPerSecond = velocityInchesPerSecond / wheelCircumference;
        return wheelRevPerSecond * ShooterConst.FLYWHEEL_TPR;
    }

    private double mapAngleToServo(double angleRadians) {
        double slope = (ShooterConst.HOOD_SERVO_MAX - ShooterConst.HOOD_SERVO_MIN)
                / (ShooterConst.HOOD_MAX_ANGLE - ShooterConst.HOOD_MIN_ANGLE);
        return slope * (angleRadians - ShooterConst.HOOD_MIN_ANGLE) + ShooterConst.HOOD_SERVO_MIN;
    }
}
