package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactIntake {

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private DcMotor FrontEaterMotor, BackEaterMotor;
    private Servo ArtifactLid;
    private State state = State.IDLE;

    public void init(HardwareMap hwMap){

        FrontEaterMotor = hwMap.get(DcMotor.class, "FE");
        BackEaterMotor = hwMap.get(DcMotor.class, "BE");
        ArtifactLid = hwMap.get(Servo.class, "AL");

        FrontEaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackEaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontEaterMotor.setPower(0);
        BackEaterMotor.setPower(0);
        ArtifactLid.setPosition(0.5);

        BackEaterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void update() {
        switch (state) {
            case INTAKING:
                setPower(0.7, 0.3, 0.5);
                break;
            case OUTTAKING:
                setPower(1, 1, 0.2);
                break;
            case IDLE:
            default:
                setPower(0, 0, 0.5);
                break;
        }
    }

    private void setPower(double frontPower, double backPower, double lidPosition){
        FrontEaterMotor.setPower(frontPower);
        BackEaterMotor.setPower(backPower);
        ArtifactLid.setPosition(lidPosition);
    }



}
