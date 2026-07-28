package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;

    GoBildaPinpoint GobildaPinpoint = new GoBildaPinpoint();

    public void init(HardwareMap hwMap){
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

        GobildaPinpoint.init(hwMap);
    }

    public Pose2D getTraditionalPose() {
        return GobildaPinpoint.getTraditionalPose();
    }

    public void MoveRobot(double y, double x, double rx){
        // 1. 센서 업데이트 및 헤딩 가져오기
        GobildaPinpoint.update();
        Pose2D position = GobildaPinpoint.ODO.getPosition();
        double botHeading = position.getHeading(AngleUnit.RADIANS);

        // 2. 필드 센트릭 회전 변환 (FTC 공식 샘플 적용)
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // 대각선 스트레이프 보정

        // 3. 모터 파워 계산 및 정규화
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        FrontLeftMotor.setPower(frontLeftPower);
        BackLeftMotor.setPower(backLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackRightMotor.setPower(backRightPower);
    }
}