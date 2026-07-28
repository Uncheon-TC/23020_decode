package org.firstinspires.ftc.teamcode.PPCalibration;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable

@TeleOp(name = "config_flywheel_pid", group = "config")
public class ShooterCalibration extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    DcMotorEx SL, SR;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double tar_vel = 0;

    private double lastP, lastI, lastD, lastF;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        timer.reset();

        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        lastP = p; lastI = i; lastD = d; lastF = f;

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
            SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // 변경된 값 기억
            lastP = p; lastI = i; lastD = d; lastF = f;
        }

        SL.setVelocity(tar_vel);
        SR.setVelocity(tar_vel);

        double cur_vel_SL = SL.getVelocity();
        double cur_vel_SR = SR.getVelocity();
        double cur_vel_avg = (cur_vel_SL + cur_vel_SR) / 2;
        double cur_vel_diff = cur_vel_SL - cur_vel_SR;


        double err_vel_SL = tar_vel - cur_vel_SL;
        double err_vel_SR = tar_vel - cur_vel_SR;
        double err_vel_avg = (err_vel_SL + err_vel_SR) / 2;
        double err_vel_diff = err_vel_SL - err_vel_SR;


        panelsTelemetry.addData("target vel", tar_vel);

        panelsTelemetry.addData("current vel(SL)", cur_vel_SL);
        panelsTelemetry.addData("current vel(SR)", cur_vel_SR);
        panelsTelemetry.addData("current vel(avg)", cur_vel_avg);
        panelsTelemetry.addData("current vel(diff, L-R)", cur_vel_diff);

        panelsTelemetry.addData("current err(SL)", err_vel_SL);
        panelsTelemetry.addData("current err(SR)", err_vel_SR);
        panelsTelemetry.addData("current err(avg)", err_vel_avg);
        panelsTelemetry.addData("current err(diff, L-R)", err_vel_diff);


        panelsTelemetry.addData("p", p);
        panelsTelemetry.addData("i", i);
        panelsTelemetry.addData("d", d);


        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        SL.setVelocity(0);
        SR.setVelocity(0);
    }
}