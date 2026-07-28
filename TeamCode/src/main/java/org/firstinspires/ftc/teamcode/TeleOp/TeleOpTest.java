package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;



@TeleOp(name = "TeleOPTest", group = "32020 OP Test")
public class TeleOpTest extends OpMode {

    org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive MecanumDrive =
            new org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive();
    org.firstinspires.ftc.teamcode.Mechanisms.ArtifactIntake ArtifactIntake =
            new org.firstinspires.ftc.teamcode.Mechanisms.ArtifactIntake();
    org.firstinspires.ftc.teamcode.Mechanisms.Shooter Shooter =
            new org.firstinspires.ftc.teamcode.Mechanisms.Shooter();

    @Override
    public void init() {

        MecanumDrive.init(hardwareMap);
        ArtifactIntake.init(hardwareMap);
        Shooter.init(hardwareMap);

/*
        follower = Constants.createFollower(hardwareMap);

        //follower.setStartingPose(startPose);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

 */

    }

    @Override
    public void loop(){

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        MecanumDrive.MoveRobot(y, x, rx);

        //=============ARTIFACT INTAKE================
        if (gamepad1.left_trigger_pressed) {
            ArtifactIntake.setPower(0.7, 0.3, 0.5);
        } else if(gamepad1.right_bumper)  {
            ArtifactIntake.setPower(1, 1, 0.0);
        } else {
            ArtifactIntake.setPower(0, 0, 0.5);
        }

        if(gamepad1.a){
            Shooter.setVelocity(2000);
        }else{
            Shooter.setVelocity(0);
        }

        // ==================== 텔레메트리 추가 부분 ====================
        Pose2D position = MecanumDrive.getTraditionalPose();
        if (position != null) {
            telemetry.addData("Heading (Degrees)", position.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading (Radians)", position.getHeading(AngleUnit.RADIANS));
        }
        telemetry.addData("curVelo", Shooter.ShooterLeft.getVelocity());

        /*
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));

         */
        telemetry.update();

        // ==========================================================
    }
}