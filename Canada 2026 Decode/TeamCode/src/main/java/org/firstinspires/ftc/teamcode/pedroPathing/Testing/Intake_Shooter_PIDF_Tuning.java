package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.ServoAngleCalculation;
import org.firstinspires.ftc.teamcode.pedroPathing._5TurnServo._5TurnServoRegulate;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

@TeleOp(name = "Intake_Shooter_PIDF_Tuning",group = "TEST")
public class Intake_Shooter_PIDF_Tuning extends OpMode {
    private Hardware hardware = new Hardware();
    private double[] tableX = {0.00, 60.93, 109.13, 151.98, 195.50, 241.03, 288.56, 331.41, 373.59, 417.78, 465.98, 506.16, 549.01, 593.19, 642.07, 686.93, 731.11, 777.98, 814.13, 858.99, 901.84, 946.03, 988.21, 1031.06, 1080.60, 1124.12, 1173.00, 1219.86, 1262.04, 1311.59, 1357.78, 1396.62, 1450.18, 1494.36, 1534.54, 1583.41, 1624.25, 1669.11, 1719.32, 1766.19, 1809.71};
    private final double[] tableY = {
            0.00,45,90.00,135, 180.00,225, 270.00,315, 360.00,405, 450.00,495, 540.00,585, 630.00,675, 720.00,765, 810.00,
            855, 900.00,945, 990.00,1035, 1080.00,1125, 1170.00,1215, 1260.00,1305, 1350.00,1395, 1440.00,1485, 1530.00,1575, 1620.00,
            1665,1710.00,1755, 1800.00
    };
    private ServoAngleCalculation servoAngleCalculation = new ServoAngleCalculation();
    private _5TurnServoRegulate turretRegulate = new _5TurnServoRegulate(tableX,tableY);
    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
        packet.put("targetRPM", Tuning_Constant.testing_Shooter_Target_RPM);
        packet.put("nothing",2800);
        Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
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

        hardware.shooter0.setVelocity(Tuning_Constant.testing_Shooter_Target_RPM /60*28);
        hardware.shooter1.setVelocity(Tuning_Constant.testing_Shooter_Target_RPM /60*28);

        hardware.intake0.setPower(gamepad1.b ? 1 : Tuning_Constant.testing_Forward_Intake_Power);
        hardware.intake1.setPower(gamepad1.b ? 1 : Tuning_Constant.testing_Rear_Intake_Power);

        hardware.angleController.setPosition(servoAngleCalculation.DegreeToPos(Tuning_Constant.angleServo));
//        hardware.turretController.setPower(turretRegulate.regulate(0.5));
        hardware.blocker.setPosition(gamepad1.b ? 0.22:0);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
        packet.put("targetRPM", Tuning_Constant.testing_Shooter_Target_RPM);
        packet.put("nothing",2800);
        Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
    }
}
