package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@TeleOp(name = "TeleOp RED", group = "32020 OP")
public class RedTeleOp extends TeleOpTest {
    @Override
    protected ShooterConst.Goal getGoal() {
        return ShooterConst.Goal.RED;
    }
}
