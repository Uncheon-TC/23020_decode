package org.firstinspires.ftc.teamcode.PPCalibration;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@Configurable

@TeleOp(name = "config_flywheel_pid", group = "config")
public class ShooterCalibration extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    DcMotorEx SL, SR;
    Servo BS;

    public static boolean flywheelEnabled = false;
    public static boolean hoodEnabled = false;
    public static double p = ShooterConst.flywheel_P;
    public static double i = ShooterConst.flywheel_I;
    public static double d = ShooterConst.flywheel_D;
    public static double f = ShooterConst.flywheel_F;

    public static double tar_vel = 0;
    public static double hoodServoPosition = 0.5;

    private double lastP, lastI, lastD, lastF;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        timer.reset();

        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        BS = hardwareMap.get(Servo.class, "BS");

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        lastP = p; lastI = i; lastD = d; lastF = f;

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelEnabled = false;
        hoodEnabled = false;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        if (p != lastP || i != lastI || d != lastD || f != lastF) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
            SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // 변경된 값 기억
            lastP = p; lastI = i; lastD = d; lastF = f;
        }

        double commandedVelocity = flywheelEnabled ? Math.max(tar_vel, 0) : 0;
        SL.setVelocity(commandedVelocity);
        SR.setPower(flywheelEnabled ? SL.getPower() : 0);
        double commandedHoodPosition = Range.clip(hoodServoPosition, 0, 1);
        if (hoodEnabled) {
            BS.setPosition(commandedHoodPosition);
        }

        double cur_vel_SL = SL.getVelocity();
        double err_vel_SL = commandedVelocity - cur_vel_SL;


        panelsTelemetry.addData("flywheel enabled", flywheelEnabled);
        panelsTelemetry.addData("target vel", commandedVelocity);
        panelsTelemetry.addData("hood enabled", hoodEnabled);
        panelsTelemetry.addData("hood position", commandedHoodPosition);

        panelsTelemetry.addData("current vel(SL)", cur_vel_SL);
        panelsTelemetry.addData("current err(SL)", err_vel_SL);
        panelsTelemetry.addData("leader power(SL)", SL.getPower());
        panelsTelemetry.addData("follower power(SR)", SR.getPower());


        panelsTelemetry.addData("p", p);
        panelsTelemetry.addData("i", i);
        panelsTelemetry.addData("d", d);
        panelsTelemetry.addData("f", f);


        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        flywheelEnabled = false;
        hoodEnabled = false;
        SL.setVelocity(0);
        SR.setPower(0);
    }
}
