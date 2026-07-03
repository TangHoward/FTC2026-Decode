package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.ServoAngleCalculation;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.ShooterCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Testing.Shooter_PIDF_Tuning;
import org.firstinspires.ftc.teamcode.pedroPathing.Turret.TurretController;
import org.firstinspires.ftc.teamcode.pedroPathing._5TurnServo._5TurnServoRegulate;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constant;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

public class Control_Mode {
    abstract static class BaseTeleOp extends OpMode {
        protected abstract boolean getIsBlue();
        private static Follower follower;

        private Hardware hardware   = new Hardware();
        private ShooterCalculator shooterCalculator = new ShooterCalculator(follower);
        private ServoAngleCalculation servoAngleCalculation;
        private TurretController turretController;
        private boolean isShooting = false;
        private static final double SHOOT_START_RPM_TOLERANCE = 50;
        private double[] tableX = {0.00, 60.93, 109.13, 151.98, 195.50, 241.03, 288.56, 331.41, 373.59, 417.78, 465.98, 506.16, 549.01, 593.19, 642.07, 686.93, 731.11, 777.98, 814.13, 858.99, 901.84, 946.03, 988.21, 1031.06, 1080.60, 1124.12, 1173.00, 1219.86, 1262.04, 1311.59, 1357.78, 1396.62, 1450.18, 1494.36, 1534.54, 1583.41, 1624.25, 1669.11, 1719.32, 1766.19, 1809.71};
        private final double[] tableY = {
                0.00,45,90.00,135, 180.00,225, 270.00,315, 360.00,405, 450.00,495, 540.00,585, 630.00,675, 720.00,765, 810.00,
                855, 900.00,945, 990.00,1035, 1080.00,1125, 1170.00,1215, 1260.00,1305, 1350.00,1395, 1440.00,1485, 1530.00,1575, 1620.00,
                1665,1710.00,1755, 1800.00
        };
        private _5TurnServoRegulate turretRegulate = new _5TurnServoRegulate(tableX,tableY);
        @Override
        public void init() {
            servoAngleCalculation = new ServoAngleCalculation();
            hardware.init(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            shooterCalculator.setPreferredAngle(farZone(follower.getPose()) ? 54 : 56);
            if(Auto_Mode.robotPose != null && Auto_Mode.turretAngle != null) {
                follower.setStartingPose(Auto_Mode.robotPose);
            }else {
                follower.setStartingPose(new Pose(getIsBlue() ? (144-96) : 96, 72, Math.toRadians(90)));
                hardware.rev9AxisImu.resetYaw();
            }
            follower.update();

            shooterCalculator.setGoal(getIsBlue() ? 2:142, 142, 44);
            turretController = new TurretController(hardware, follower);
            turretController.setAimPoint(  getIsBlue() ? 4:140,140);
            turretController.setTarget(getIsBlue() ? TurretController.Target.ID_20 : TurretController.Target.ID_24);
            turretController.setAimMode(TurretController.AimMode.IMU_PID);
        }

        @Override
        public void init_loop() {
            drawOnlyCurrent();
            follower.update();
        }

        @Override
        public void start() {
            follower.startTeleopDrive();
            follower.update();
            //hardware.limelight.pipelineSwitch(0);
            //hardware.limelight.start();
        }

        @Override
        public void loop() {
            shooterCalculator.setPreferredAngle(farZone(follower.getPose()) ? 54 : 56);
            turretController.setTxTarget((farZone(follower.getPose()) ? -15 : 0) * (getIsBlue() ? -1 : 1));
            ShooterCalculator.ShootResult shooterResult = shooterCalculator.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1),
                    -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1),
                    -gamepad1.right_stick_x,false);

            if(shooterResult.valid) {
            hardware.shooter0.setVelocity(shooterResult.flywheelRPM/*Tuning_Constant.testing_Shooter_Target_RPM*/ / 60 * 28);
            hardware.shooter1.setVelocity(shooterResult.flywheelRPM/*Tuning_Constant.testing_Shooter_Target_RPM*/ / 60 * 28);
            hardware.angleController.setPosition(servoAngleCalculation.DegreeToPos(Math.toDegrees(shooterResult.launchAngle)));
            turretController.setLastBaseSpeed(shooterResult.launchSpeed);
            turretController.setLastBaseAngle(shooterResult.launchAngle);
            }
            double rpmError = shooterResult.flywheelRPM - hardware.shooter0.getVelocity() * 60 / 28;

            if (!gamepad1.b) {
                isShooting = false;
            } else if (!isShooting) {
                if (rpmError > -0 && rpmError < 100) {
                    isShooting = true;
                }
            }
            if(gamepad1.x){
                turretController.relocalizeWithLimelight();
            }
            if(gamepad1.yWasReleased()){
                turretController.setAimMode(
                        turretController.getAimMode() == TurretController.AimMode.APRIL_TAG
                                ? TurretController.AimMode.IMU_PID
                                : TurretController.AimMode.APRIL_TAG
                );
            }
            hardware.intake0.setPower(isShooting ? 0.6 : Tuning_Constant.testing_Forward_Intake_Power);
            hardware.intake1.setPower(isShooting ? 0.6 : Tuning_Constant.testing_Rear_Intake_Power);
            //hardware.turretController.setPosition(turretRegulate.regulate(0.5));
            hardware.blocker.setPosition(isShooting ? 0.22 : 0);

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
            turretController.update();

            telemetry.addData("是否有錯誤",shooterResult.valid ?"沒有":"有");
            telemetry.addData("射球計算機錯誤訊息",shooterResult.errorMessage);
            telemetry.addData("設定RPM", shooterResult.flywheelRPM);
            telemetry.addData("設定角度", Math.toDegrees(shooterResult.launchAngle));
            telemetry.addData("設定位置", servoAngleCalculation.DegreeToPos(Math.toDegrees(shooterResult.launchAngle)));
            telemetry.addData("砲台角度",  turretController.getCurrentAngleDeg());
            telemetry.addData("目標角度",  turretController.getTargetAngleDeg());
            telemetry.addData("Robot X", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
            telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("旋轉砲台朝向", hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);
            telemetry.addData("Tx是否套用", turretController.isLastTxApplied());
            telemetry.addData("Tx補償角度", turretController.getLastTxCorrectionDeg());
            telemetry.addData("射球誤差",Math.abs(shooterResult.flywheelRPM - hardware.shooter0.getVelocity() * 60/28));
            follower.update();
            drawOnlyCurrent();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("targetRPM", shooterResult.flywheelRPM);
            Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
        }


        public static void drawOnlyCurrent() {
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
        }
        public static boolean farZone(Pose robotPose){
            return robotPose.getY() < 48;
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
