package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpTest;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

public class MecanumDrive {

    TeleOpTest teleOpTest = new TeleOpTest() {
        @Override
        protected ShooterConst.Goal getGoal() {
            return null;
        }
    };
    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private double driverHeadingRadians;


    GoBildaPinpoint GobildaPinpoint = new GoBildaPinpoint();

    public void init(HardwareMap hwMap, Pose2D startingPosition){
        FrontLeftMotor = hwMap.get(DcMotor.class, "FL");
        FrontRightMotor = hwMap.get(DcMotor.class, "FR");
        BackLeftMotor = hwMap.get(DcMotor.class, "BL");
        BackRightMotor = hwMap.get(DcMotor.class, "BR");

        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GobildaPinpoint.init(hwMap, startingPosition);
    }

    public Pose2D getTraditionalPose() {
        return GobildaPinpoint.getTraditionalPose();
    }

    public void setPosition(Pose2D position) {
        GobildaPinpoint.setPosition(position);
    }

    public void setDriverHeadingDegrees(double headingDegrees) {
        driverHeadingRadians = Math.toRadians(headingDegrees);
    }

    public void MoveRobot(double y, double x, double rx){

        double slow = 1 - (0.8 * teleOpTest.gamepad1.left_trigger);

        // 1. 센서 업데이트 및 헤딩 가져오기
        GobildaPinpoint.update();
        Pose2D position = GobildaPinpoint.ODO.getPosition();
        double botHeading = position.getHeading(AngleUnit.RADIANS);

        // 2. 플레이어 시점을 반영한 필드 센트릭 회전 변환
        double fieldRotation = driverHeadingRadians - botHeading;
        double rotX = x * Math.cos(fieldRotation) - y * Math.sin(fieldRotation);
        double rotY = x * Math.sin(fieldRotation) + y * Math.cos(fieldRotation);

        rotX = rotX * 1.1;  // 대각선 스트레이프 보정

        // 3. 모터 파워 계산 및 정규화
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator * slow;
        double backLeftPower   = (rotY - rotX + rx) / denominator * slow;
        double frontRightPower = (rotY - rotX - rx) / denominator * slow;
        double backRightPower  = (rotY + rotX - rx) / denominator * slow;

        FrontLeftMotor.setPower(frontLeftPower);
        BackLeftMotor.setPower(backLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackRightMotor.setPower(backRightPower);
    }
}
