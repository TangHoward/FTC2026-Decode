package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

@TeleOp(name = "ShooterPIDFTuning", group = "TEST")
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
        packet.put("targetRPM", Tuning_Constant.testing_Shooter_Target_RPM);

        Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        hardware.shooter0.setVelocityPIDFCoefficients(
                Tuning_Constant.Shooter_P_Far
                ,Tuning_Constant.Shooter_I_Far
                ,Tuning_Constant.Shooter_D_Far
                ,Tuning_Constant.Shooter_F_Far);
        hardware.shooter1.setVelocityPIDFCoefficients(
                Tuning_Constant.Shooter_P_Far
                ,Tuning_Constant.Shooter_I_Far
                ,Tuning_Constant.Shooter_D_Far
                ,Tuning_Constant.Shooter_F_Far);

        hardware.intake0.setPower(1);
        hardware.intake1.setPower(-0.1);
    }

    @Override
    public void loop() {

        hardware.shooter0.setVelocity(Tuning_Constant.testing_Shooter_Target_RPM *28/60);
        hardware.shooter1.setVelocity(Tuning_Constant.testing_Shooter_Target_RPM *28/60);
    }
}
