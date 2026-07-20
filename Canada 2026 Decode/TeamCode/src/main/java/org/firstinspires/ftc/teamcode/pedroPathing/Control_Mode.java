package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        protected abstract boolean getIsFar();
        private boolean isFar = getIsFar();
        private static Follower follower;

        private Hardware hardware   = new Hardware();
        private ShooterCalculator shooterCalculator;
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
            if(Auto_Mode.robotPose != null && Auto_Mode.turretAngle != null) {
                follower.setStartingPose(Auto_Mode.robotPose);
                Configurable_Constant.turretAngleOffset = getIsBlue() ? 180: 0;
            }else {
                Pose closeStartingPose = new Pose(
                        (getIsBlue()? 144 -107.9 : 107.9),
                        132.7
                        ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
                );
                Pose farStartingPose = new Pose(
                        (getIsBlue()? 144 - 96.7  : 96.7),
                        9.3
                        ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
                );
                Configurable_Constant.turretAngleOffset = getIsBlue() ? 180: 0;
                follower.setPose(getIsFar() ? farStartingPose : closeStartingPose);
                hardware.rev9AxisImu.resetYaw();
            }
            follower.update();
            shooterCalculator = new ShooterCalculator(follower);

            shooterCalculator.setGoal(getIsBlue() ? 2:142, 142, 44);
            turretController = new TurretController(hardware, follower);
            turretController.setAimPoint(getIsBlue() ? 2:142,142);
            turretController.setTarget(getIsBlue() ? TurretController.Target.ID_20 : TurretController.Target.ID_24);
//            turretController.setAimMode(TurretController.AimMode.APRIL_TAG);

            isFar = getIsFar();
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

            if(gamepad1.rightBumperWasPressed()){
                isFar = !getIsBlue();
            } else if (gamepad1.leftBumperWasPressed()) {
                isFar = getIsBlue();
            }
            shooterCalculator.setFarMode(isFar);
            turretController.setTxTarget( (isFar ? -1.5 : 0) * (getIsBlue() ? -1 : 1));
            ShooterCalculator.ShootResult shooterResult = shooterCalculator.update();

            // 按住 X：停下來重新定位。搖桿輸入歸零、讓機器人真正靜止，
            // 品質過濾（角速度 / 平移速度）通過後才會採用視覺讀值並修正位置。
            // 放開 X 才恢復正常搖桿駕駛。
            boolean relocalizing = gamepad1.x;
            if (relocalizing) {
                follower.setTeleOpDrive(0, 0, 0, false);
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1)*1,
                        -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1)*1,
                        -gamepad1.right_stick_x*0.6, false);
            }

            if(shooterResult.valid) {
                turretController.setLastBaseSpeed(shooterResult.launchSpeed);
                turretController.setLastBaseAngle(shooterResult.launchAngleRad);
                if(!gamepad1.right_trigger_pressed) {
                    hardware.shooter0.setVelocity(shooterResult.flywheelRPM / 60 * 28);
                    hardware.shooter1.setVelocity(shooterResult.flywheelRPM / 60 * 28);
                }else if (!isShooting){
                    hardware.shooter0.setPower(-0);
                    hardware.shooter1.setPower(-0);}
                hardware.angleController.setPosition(servoAngleCalculation.DegreeToPos(shooterResult.servoAngleDeg));
            }

            double rpmError = shooterResult.flywheelRPM- hardware.shooter0.getVelocity() * 60 / 28;

            if (!gamepad1.b) {
                isShooting = false;
            } else if (!isShooting) {
                if (rpmError > -50 && rpmError < 100 && ((turretController.getAimMode() == TurretController.AimMode.APRIL_TAG) ?
                        (Math.abs(turretController.getLastTxErrorDeg()) < (isFar ? 2 :3)): true)) {
                    isShooting = true;
                }
            }
            if (relocalizing) {
                turretController.relocalizeStationary();
            }
            if(gamepad1.yWasReleased()){
                turretController.setAimMode(
                        turretController.getAimMode() == TurretController.AimMode.APRIL_TAG
                                ? TurretController.AimMode.IMU_PID
                                : TurretController.AimMode.APRIL_TAG
                );
            }
            hardware.intake0.setPower(isShooting ? 1 * (isFar ?  Tuning_Constant.testing_Forward_Intake_Power : 1): 1
                            * (gamepad1.dpad_down ? -1 : 1));
            hardware.intake1.setPower(isShooting ? 1 * (isFar ?  Tuning_Constant.testing_Rear_Intake_Power : 1): 0.4
                            * (gamepad1.dpad_down ? -1 : 1));
            //hardware.turretController.setPosition(turretRegulate.regulate(0.5));
            hardware.blocker.setPosition(isShooting ? 0.22 : 0);

            if(isFar) {
                hardware.shooter0.setVelocityPIDFCoefficients(
                        Tuning_Constant.Shooter_P_Far
                        , Tuning_Constant.Shooter_I_Far
                        , Tuning_Constant.Shooter_D_Far
                        , Tuning_Constant.Shooter_F_Far);
                hardware.shooter1.setVelocityPIDFCoefficients(
                        Tuning_Constant.Shooter_P_Far
                        , Tuning_Constant.Shooter_I_Far
                        , Tuning_Constant.Shooter_D_Far
                        , Tuning_Constant.Shooter_F_Far);
            }else {
                hardware.shooter0.setVelocityPIDFCoefficients(
                        Tuning_Constant.Shooter_P_Close
                        ,Tuning_Constant.Shooter_I_Close
                        ,Tuning_Constant.Shooter_D_Close
                        ,Tuning_Constant.Shooter_F_Close);
                hardware.shooter1.setVelocityPIDFCoefficients(
                        Tuning_Constant.Shooter_P_Close
                        ,Tuning_Constant.Shooter_I_Close
                        ,Tuning_Constant.Shooter_D_Close
                        ,Tuning_Constant.Shooter_F_Close);
            }
            turretController.update(relocalizing, isFar);

            telemetry.addData("是否有錯誤",shooterResult.valid ?"沒有":"有");
            telemetry.addData("射球計算機錯誤訊息",shooterResult.errorMessage);
            telemetry.addData("設定RPM", shooterResult.flywheelRPM);
            telemetry.addData("設定角度", shooterResult.servoAngleDeg);
            telemetry.addData("設定位置", servoAngleCalculation.DegreeToPos(shooterResult.servoAngleDeg));
            telemetry.addData("砲台角度",  turretController.getCurrentAngleDeg());
            telemetry.addData("目標角度",  turretController.getTargetAngleDeg());
            telemetry.addData("Robot X", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
            telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("旋轉砲台朝向", hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);
            telemetry.addData("Tx原始誤差", turretController.getLastTxErrorDeg());
            telemetry.addData("射球誤差",Math.abs(shooterResult.flywheelRPM - hardware.shooter0.getVelocity() * 60/28));
            telemetry.addData("停止重新定位中", relocalizing);
            telemetry.addData("視覺定位狀態", turretController.getLimelightRejectReason());
            telemetry.addData("砲台角速度(deg/s)", turretController.getTurretAngularVelocityDegPerSec());
            telemetry.addData("底盤角速度(deg/s)", turretController.getChassisAngularVelocityDegPerSec());
            follower.update();
            drawOnlyCurrent();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("shooter1RPM", hardware.shooter1.getVelocity() * 60/28);
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
    @TeleOp(name = "紅方近程式",group = "RedTeleOp")
    public static class RedCloseTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return false;
        }

        @Override
        protected boolean getIsFar() {
            return false;
        }
    }
    @TeleOp(name = "紅方遠程式",group = "RedTeleOp")
    public static class RedFarTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return false;
        }

        @Override
        protected boolean getIsFar() {
            return true;
        }
    }
    @TeleOp(name = "藍方近程式",group = "BlueTeleOp")
    public static class BlueCloseTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return true;
        }

        @Override
        protected boolean getIsFar() {
            return false;
        }
    }
    @TeleOp(name = "藍方遠程式",group = "BlueTeleOp")
    public static class BlueFarTeleOp extends BaseTeleOp{
        @Override
        protected boolean getIsBlue() {
            return true;
        }

        @Override
        protected boolean getIsFar() {
            return true;
        }
    }
}