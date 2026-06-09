package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constant;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

@TeleOp(name = "example_structure", group = "example1")
public class Shooter_PIDF_Tuning extends OpMode {
    private Hardware hardware = new Hardware();
    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double motorspeed = 3500;
    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
        packet.put("targetRPM", Configurable_Constant.test_motor_Speed);
        Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        hardware.shooter0.setVelocityPIDFCoefficients(
                Tuning_Constant.Shooter_P
                ,Tuning_Constant.Shooter_I
                ,Tuning_Constant.Shooter_D
                ,Tuning_Constant.Shooter_F);
        hardware.shooter1.setVelocityPIDFCoefficients(
                Tuning_Constant.Shooter_P
                ,Tuning_Constant.Shooter_I
                ,Tuning_Constant.Shooter_D
                ,Tuning_Constant.Shooter_F);
    }

    @Override
    public void loop() {

        hardware.shooter0.setVelocity(Configurable_Constant.test_motor_Speed*28/60);
        hardware.shooter1.setVelocity(Configurable_Constant.test_motor_Speed*28/60);
    }
}
