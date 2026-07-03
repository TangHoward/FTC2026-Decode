package org.firstinspires.ftc.teamcode.pedroPathing.Testing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMU Test")
public class IMUtest extends LinearOpMode {

    public Rev9AxisImu imu;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(Rev9AxisImu.class, "revIMU");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            telemetry.addData("Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}