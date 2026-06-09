package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.ShooterCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

public class Control_Mode {
    abstract static class BaseTeleOp extends OpMode {
        protected abstract boolean getIsBlue();
        private static Follower follower;

        private Hardware hardware = new Hardware();
        private ShooterCalculator shooterCalculator;

        @Override
        public void init() {
            hardware.init(hardwareMap);

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            follower.update();
            shooterCalculator = new ShooterCalculator(follower);

            shooterCalculator.setGoal(144, 144, 45);
        }

        @Override
        public void init_loop() {
            drawOnlyCurrent();
            follower.update();

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
        public void start() {
            follower.startTeleopDrive();
        }

        @Override
        public void loop() {
            ShooterCalculator.ShootResult shooterResult = shooterCalculator.update();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1),
                    -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1),
                    -gamepad1.right_stick_x,false);
            if(shooterResult.valid) {
                hardware.shooter0.setVelocity(shooterResult.flywheelRPM / 60 * 28);
                hardware.shooter1.setVelocity(shooterResult.flywheelRPM / 60 * 28);
            }
            hardware.intake0.setPower(1);

            telemetry.addData("是否有錯誤:",shooterResult.valid ?"沒有":"有");
            telemetry.addData("射球計算機錯誤訊息",shooterResult.errorMessage);
            telemetry.addData("設定RPM", shooterResult.flywheelRPM);
            telemetry.update();

        }

        public static void drawOnlyCurrent() {
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
        }
    }
    @TeleOp(name = "紅方程式",group = "TeleOp")
    public static class RedTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return false;
        }
    }
    @TeleOp(name = "藍方程式",group = "TeleOp")
    public static class BlueTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return true;
        }
    }
}
