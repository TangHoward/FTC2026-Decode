package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;


@TeleOp()
public class TuningFlyWheel extends OpMode {

    Hardware hardware = new Hardware();
    TelemetryManager telemetryM;
    private boolean  status =true;

    @Override
    public void init() {
        hardware.init(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
    }
    @Override
    public void loop() {
        status = true;

        if(status){

            hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_longlunch_KP,0,0, Configurable_Constants.shooter_longlunch_F);
            telemetry.addData("FlyWheel", "遠距離模式(高轉速模式)");
            if(gamepad1.dpadUpWasPressed()){
                hardware.shooter.setVelocity(Configurable_Constants.shooterLongRangeSpeed/60*28);
            }else if(gamepad1.dpadDownWasPressed()){
                hardware.shooter.setVelocity(0);
            }
        }else{
            hardware.shooter.setVelocity(Configurable_Constants.shooterNearRangeSpeed /60*28);
            hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_nearlunch_KP,0,0, Configurable_Constants.shooter_nearlunch_F);
            telemetry.addData("FlyWheel", "近距離模式(低轉速模式)");
        }

        telemetry.addData("FlyWheel.Velocity", hardware.shooter.getVelocity()*60/28);
        telemetry.addData("targetspeed", Configurable_Constants.shooterLongRangeSpeed);
        telemetry.update();
        telemetryM.update(telemetry);
    }
}
