package org.firstinspires.ftc.teamcode.PPCalibration;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Turret;
import org.firstinspires.ftc.teamcode.subConstant.ShooterConst;

@Configurable
@TeleOp(name = "config_turret_pid", group = "config")
public class TurretCalibration extends OpMode {
    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();
    private final Turret turret = new Turret();

    // 패널에서 enabled를 true로 바꿔야 터렛이 움직인다.
    public static boolean enabled = false;
    public static double targetAngleDegrees = 0;

    public static double p = ShooterConst.turret_P;
    public static double i = ShooterConst.turret_I;
    public static double d = ShooterConst.turret_D;
    public static double f = ShooterConst.turret_F;

    public static double maxPower = ShooterConst.TURRET_MAX_POWER;
    public static double deadbandDegrees =
            Math.toDegrees(ShooterConst.TURRET_DEADBAND_RADIANS);
    public static double gearRatio = ShooterConst.TURRET_GEAR_RATIO;

    private double previousGearRatio = gearRatio;

    @Override
    public void init() {
        applyPanelValues();
        turret.init(hardwareMap);
        // 캘리브레이션을 시작할 때 놓인 방향을 0도로 사용한다.
        turret.resetAngleTracking();
        enabled = false;
    }

    @Override
    public void loop() {
        applyPanelValues();

        if (gearRatio != previousGearRatio) {
            previousGearRatio = gearRatio;
            turret.resetAngleTracking();
        }

        if (enabled) {
            turret.trackAngle(Math.toRadians(targetAngleDegrees));
        } else {
            turret.stop();
        }

        panelsTelemetry.addData("enabled", enabled);
        panelsTelemetry.addData("target angle (deg)", targetAngleDegrees);
        panelsTelemetry.addData("current angle (deg)", turret.getCurrentAngleDegrees());
        panelsTelemetry.addData("angle error (deg)", turret.getAngleErrorDegrees());
        panelsTelemetry.addData("servo power", turret.getServoPower());
        panelsTelemetry.addData("encoder voltage", turret.getEncoderVoltage());
        panelsTelemetry.addData("gear ratio", gearRatio);
        panelsTelemetry.addData("P", p);
        panelsTelemetry.addData("I", i);
        panelsTelemetry.addData("D", d);
        panelsTelemetry.addData("F", f);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        enabled = false;
        turret.stop();
    }

    private void applyPanelValues() {
        ShooterConst.turret_P = p;
        ShooterConst.turret_I = i;
        ShooterConst.turret_D = d;
        ShooterConst.turret_F = f;
        ShooterConst.TURRET_MAX_POWER = Math.min(Math.abs(maxPower), 1.0);
        ShooterConst.TURRET_DEADBAND_RADIANS =
                Math.toRadians(Math.abs(deadbandDegrees));
        ShooterConst.TURRET_GEAR_RATIO = Math.max(Math.abs(gearRatio), 0.001);
    }
}
