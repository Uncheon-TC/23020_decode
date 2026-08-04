package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@TeleOp(name = "TeleOp BLUE", group = "32020 OP")
public class BlueTeleOp extends TeleOpTest {
    @Override
    protected ShooterConst.Goal getGoal() {
        return ShooterConst.Goal.BLUE;
    }
}
