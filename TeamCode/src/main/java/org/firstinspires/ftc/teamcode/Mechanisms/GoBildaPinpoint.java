package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class GoBildaPinpoint {

    GoBildaPinpointDriver ODO;

    public void init(HardwareMap hwMap, Pose2D startingPosition){
        ODO = hwMap.get(GoBildaPinpointDriver.class, "odo");

        //ODO.setOffsets(,);
        ODO.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        ODO.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);


        //=======TRADITIONAL FTC COORDINATE============
        ODO.resetPosAndIMU();
        ODO.setPosition(startingPosition);

    }

    public void update() {
        ODO.update();
    }

    public void setPosition(Pose2D position) {
        ODO.setPosition(position);
        ODO.update();
    }

    public Pose2D getTraditionalPose() {
        return ODO.getPosition();
    }

}
