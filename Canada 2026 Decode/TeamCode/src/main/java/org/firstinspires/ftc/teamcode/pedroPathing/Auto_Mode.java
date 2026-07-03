package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;

import static org.firstinspires.ftc.teamcode.pedroPathing.Control_Mode.BaseTeleOp.farZone;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.ServoAngleCalculation;
import org.firstinspires.ftc.teamcode.pedroPathing.Testing.Shooter_PIDF_Tuning;
import org.firstinspires.ftc.teamcode.pedroPathing.Turret.TurretController;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constant;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

public class Auto_Mode {

    public static Pose robotPose = null;
    public static Double turretAngle = null;
    abstract static class BaseCloseAuto extends OpMode{
        protected abstract boolean getIsBlue();
        private static Follower follower;
        private Hardware hardware   = new Hardware();
        private ServoAngleCalculation servoAngleCalculation = new ServoAngleCalculation();
        private TurretController turretController;
        private Pose startingPose,scoringPose,intake1st,intake2nd,gatePose,
                intake1stControl,intake2ndControl, gateControl;
        private PathChain goStraightScoringPose, intake1stPath, intake2ndPath, gotoGate , goBackGate;
        public void initPosePoint(){
            startingPose = new Pose(
                    (getIsBlue()? 144 -109.8 : 109.8),
                    133.4
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            scoringPose = new Pose(
                    (getIsBlue()? 144 - 90 : 90),
                    84
                    ,Math.toRadians(Math.abs(45 - (getIsBlue() ? 180 : 0)))
            );
            intake1st = new Pose(
                    (getIsBlue()? 144 - 115: 115),
                    84
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            intake2nd = new Pose(
                    (getIsBlue()? 144 - 115 : 115),
                    60
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            intake2ndControl = new Pose(
                    (getIsBlue()? 144 -97 :97) ,
                    60
            );
            gatePose = new Pose(
                    (getIsBlue()? 144 - 127.7:127.7),
                    61.1
                    ,Math.toRadians(Math.abs(25.5 - (getIsBlue() ? 180 : 0)))
            );
            gateControl = new Pose(
                    (getIsBlue()? 144 - 111.2:111.2),
                    61.1
                    ,Math.toRadians(Math.abs(25.5 - (getIsBlue() ? 180 : 0)))
            );
        }
        public void buildPath(){
            goStraightScoringPose = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose,scoringPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,scoringPose.getHeading(),0.5))
                    .build();
            intake1stPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, intake1st))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, intake1st.getHeading(), 0.2))
                    .addPath(new BezierLine(intake1st, scoringPose))
                    .setLinearHeadingInterpolation(intake1st.getHeading(), scoringPose.getHeading())
                    .build();
            intake2ndPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, intake2ndControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, intake2ndControl.getHeading(), 1))
                    .addPath(new BezierLine(intake2ndControl, intake2nd))
                    .setLinearHeadingInterpolation(intake2ndControl.getHeading(), intake2nd.getHeading())
                    .addPath(new BezierLine(intake2nd, scoringPose))
                    .setLinearHeadingInterpolation(intake2nd.getHeading(), scoringPose.getHeading())
                    .build();
            gotoGate = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateControl.getHeading(), 1))
                    .addPath(new BezierLine(gateControl,gatePose))
                    .setLinearHeadingInterpolation(gateControl.getHeading(),gatePose.getHeading(),0)
                    .build();
            goBackGate = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateControl.getHeading(), 1))
                    .addPath(new BezierLine(gateControl,scoringPose))
                    .setLinearHeadingInterpolation(gateControl.getHeading(),scoringPose.getHeading(),0.5)
                    .build();
        }
        public Command autoRoutine(){
            return sequential(
                    instant(() -> setShooterRPM(3500)),
                    instant(() -> setServoAngle(50)),
                    follow(follower, goStraightScoringPose, true),
                    shooting(),
                    follow(follower, intake2ndPath, true),
                    shooting(),
                    follow(follower, gotoGate, true),
                    waitMs(2000),
                    follow(follower, goBackGate),
                    shooting(),
                    follow(follower, gotoGate, true),
                    waitMs(2000),
                    follow(follower, goBackGate),
                    shooting(),
                    follow(follower, intake1stPath, true),
                    shooting()
            );
        }

        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            Configurable_Constant.turretAngleOffset = getIsBlue() ? 180: 0;
            Scheduler.reset();
            hardware.init(hardwareMap);
            hardware.rev9AxisImu.resetYaw();
            initPosePoint();
            follower.setStartingPose(startingPose);
            buildPath();
            follower.update();
            turretController = new TurretController(hardware, follower);
            turretController.setAimPoint(  getIsBlue() ? 4:140,140);
            turretController.setTarget(getIsBlue() ? TurretController.Target.ID_20 : TurretController.Target.ID_24);
            turretController.setAimMode(TurretController.AimMode.IMU_PID);
            hardware.shooter0.setVelocityPIDFCoefficients(
                    Tuning_Constant.Shooter_P
                    ,0
                    ,Tuning_Constant.Shooter_D
                    ,Tuning_Constant.Shooter_F);
            hardware.shooter1.setVelocityPIDFCoefficients(
                    Tuning_Constant.Shooter_P
                    ,0
                    ,Tuning_Constant.Shooter_D
                    ,Tuning_Constant.Shooter_F);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("targetRPM", 3500);
            Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
        }

        @Override
        public void start() {
            hardware.intake0.setPower(Tuning_Constant.testing_Forward_Intake_Power);
            hardware.intake1.setPower(Tuning_Constant.testing_Rear_Intake_Power);
            hardware.blocker.setPosition(0);
            schedule(autoRoutine());
        }

        @Override
        public void loop() {
            follower.update();
            turretController.update();
            turretController.setTxTarget((farZone(follower.getPose()) ? -15 : 0) * (getIsBlue() ? -1 : 1));
            Scheduler.execute();
            robotPose = follower.getPose();
            turretAngle = hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("targetRPM", 3500);
            Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
        }
        public void setServoAngle(double angle){
            hardware.angleController.setPosition(servoAngleCalculation.DegreeToPos(angle));
        }
        public void setShooterRPM(double RPM, boolean onOff){
            RPM = RPM /60 * 28;
            if(onOff) {
                hardware.shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.shooter0.setVelocity(RPM);
                hardware.shooter1.setVelocity(RPM);
            }else{
                hardware.shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.shooter0.setPower(0);
                hardware.shooter1.setPower(0);
            }
        }
        public void setShooterRPM(double RPM){
            RPM = RPM /60 * 28;
            hardware.shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.shooter0.setVelocity(RPM);
            hardware.shooter1.setVelocity(RPM);
        }
        public Command shooting(){
            return sequential(
                    waitMs(500),
                    instant(() -> hardware.intake0.setPower(0.6)),
                    instant(() -> hardware.intake1.setPower(0.6)),
                    instant(() -> hardware.blocker.setPosition(0.22)),
                    waitMs(700),
                    instant(() -> hardware.intake0.setPower(Tuning_Constant.testing_Forward_Intake_Power)),
                    instant(() -> hardware.intake1.setPower(Tuning_Constant.testing_Rear_Intake_Power)),
                    instant(() -> hardware.blocker.setPosition(0))
            );
        }

    }

    @Autonomous(name = "紅方近自動", group ="Close")
    public static class RedCloseAuto extends BaseCloseAuto{
        @Override
        protected boolean getIsBlue() {
            return false;
        }
    }
    abstract static class BaseFarAuto extends OpMode{
        protected abstract boolean getIsBlue();
        private static Follower follower;
        private Hardware hardware   = new Hardware();
        private ServoAngleCalculation servoAngleCalculation = new ServoAngleCalculation();
        private TurretController turretController;
        private Pose startingPose,scoringPose, intake3rd,gatePose,loadZonePose,loadZoneTwo,
                intake3rdControl, gateControl;
        private PathChain goStraightScoringPose, intake3rdPath, gotoGate , goBackGate
                , goStraightLoadZone, goGoStraightLoadZoneTwo;
        public void initPosePoint(){
            startingPose = new Pose(
                    (getIsBlue()? 144 - 96  : 96),
                    10
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            scoringPose = new Pose(
                    (getIsBlue()? 144 - 94 : 94),
                    14
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            intake3rd = new Pose(
                    (getIsBlue()? 144 - 1115: 115),
                    35
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            intake3rdControl = new Pose(
                    (getIsBlue()? 144 - 100: 100),
                    35
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            gatePose = new Pose(
                    (getIsBlue()? 144 - 127.7:127.7),
                    61.1
                    ,Math.toRadians(Math.abs(25.5 - (getIsBlue() ? 180 : 0)))
            );
            gateControl = new Pose(
                    (getIsBlue()? 144 - 111.2:111.2),
                    61.1
                    ,Math.toRadians(Math.abs(25.5 - (getIsBlue() ? 180 : 0)))
            );
            loadZonePose = new Pose(
                    (getIsBlue()? 144 - 130:130),
                    12
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
            loadZoneTwo = new Pose(
                    (getIsBlue()? 144 - 128:128),
                    35
                    ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
            );
        }
        public void buildPath(){
            goStraightScoringPose = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose,scoringPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,scoringPose.getHeading(),0.5))
                    .build();
            intake3rdPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, intake3rdControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, intake3rdControl.getHeading(), 0.2))
                    .addPath(new BezierLine(intake3rdControl, intake3rd))
                    .setLinearHeadingInterpolation(intake3rdControl.getHeading(), intake3rd.getHeading())
                    .addPath(new BezierLine(intake3rd, scoringPose))
                    .setLinearHeadingInterpolation(intake3rd.getHeading(), scoringPose.getHeading())
                    .build();
            gotoGate = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateControl.getHeading(), 1))
                    .addPath(new BezierLine(gateControl,gatePose))
                    .setLinearHeadingInterpolation(gateControl.getHeading(),gatePose.getHeading(),0)
                    .build();
            goBackGate = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateControl))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateControl.getHeading(), 1))
                    .addPath(new BezierLine(gateControl,scoringPose))
                    .setLinearHeadingInterpolation(gateControl.getHeading(),scoringPose.getHeading(),0.5)
                    .build();
            goStraightLoadZone = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose,loadZonePose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,loadZonePose.getHeading(),0.5))
                    .build();
            goGoStraightLoadZoneTwo = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose,loadZoneTwo))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,loadZoneTwo.getHeading(),0.5))
                    .build();
        }
        public Command autoRoutine(){
            return sequential(
                    instant(() -> setShooterRPM(3500)),
                    instant(() -> setServoAngle(52)),
                    follow(follower, goStraightScoringPose, true)
                    ,waitMs(600)
                    ,shooting()
                    ,follow(follower, intake3rdPath,true)
                    ,shooting()
                    ,follow(follower, goStraightLoadZone,true)
                    ,waitMs(200)
                    ,shaking()
                    ,follow(follower,goStraightScoringPose,true)
                    ,shooting()
                    ,follow(follower, goGoStraightLoadZoneTwo,true)
                    ,waitMs(200)
                    ,shaking()
                    ,follow(follower,goStraightScoringPose,true)
                    ,shooting()
                    ,follow(follower, goStraightLoadZone,true)
                    ,waitMs(200)
                    ,shaking()
                    ,follow(follower,goStraightScoringPose,true)
                    ,shooting()
                    ,follow(follower, goGoStraightLoadZoneTwo,true)
                    ,waitMs(200)
                    ,shaking()
                    ,follow(follower,goStraightScoringPose,true)
                    ,waitMs(700)
                    ,shooting()
                    ,follow(follower, goStraightLoadZone,true)
            );
        }

        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            Configurable_Constant.turretAngleOffset = getIsBlue() ? 180: 0;
            Scheduler.reset();
            hardware.init(hardwareMap);
            hardware.rev9AxisImu.resetYaw();
            initPosePoint();
            follower.setStartingPose(startingPose);
            buildPath();
            follower.update();
            turretController = new TurretController(hardware, follower);
            turretController.setAimPoint(  getIsBlue() ? 4:140,140);
            turretController.setTarget(getIsBlue() ? TurretController.Target.ID_20 : TurretController.Target.ID_24);
            turretController.setAimMode(TurretController.AimMode.IMU_PID);
            hardware.shooter0.setVelocityPIDFCoefficients(
                    Tuning_Constant.Shooter_P
                    ,0
                    ,Tuning_Constant.Shooter_D
                    ,Tuning_Constant.Shooter_F);
            hardware.shooter1.setVelocityPIDFCoefficients(
                    Tuning_Constant.Shooter_P
                    ,0
                    ,Tuning_Constant.Shooter_D
                    ,Tuning_Constant.Shooter_F);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("targetRPM", 3500);
            Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
        }

        @Override
        public void start() {
            hardware.intake0.setPower(Tuning_Constant.testing_Forward_Intake_Power);
            hardware.intake1.setPower(Tuning_Constant.testing_Rear_Intake_Power);
            hardware.blocker.setPosition(0);
            schedule(autoRoutine());
        }

        @Override
        public void loop() {
            follower.update();
            turretController.update();
            turretController.setTxTarget((farZone(follower.getPose()) ? -15 : 0) * (getIsBlue() ? -1 : 1));
            Scheduler.execute();
            robotPose = follower.getPose();
            turretAngle = hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter0.getVelocity() * 60/28);
            packet.put("targetRPM", 3500);
            Shooter_PIDF_Tuning.dashboard.sendTelemetryPacket(packet);
        }
        public void setServoAngle(double angle){
            hardware.angleController.setPosition(servoAngleCalculation.DegreeToPos(angle));
        }
        public void setShooterRPM(double RPM, boolean onOff){
            RPM = RPM /60 * 28;
            if(onOff) {
                hardware.shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.shooter0.setVelocity(RPM);
                hardware.shooter1.setVelocity(RPM);
            }else{
                hardware.shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.shooter0.setPower(0);
                hardware.shooter1.setPower(0);
            }
        }
        public void setShooterRPM(double RPM){
            RPM = RPM /60 * 28;
            hardware.shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.shooter0.setVelocity(RPM);
            hardware.shooter1.setVelocity(RPM);
        }
        public Command shooting(){
            return sequential(
                    waitMs(700),
                    instant(() -> hardware.intake0.setPower(0.6)),
                    instant(() -> hardware.intake1.setPower(0.6)),
                    instant(() -> hardware.blocker.setPosition(0.22)),
                    waitMs(700),
                    instant(() -> hardware.intake0.setPower(Tuning_Constant.testing_Forward_Intake_Power)),
                    instant(() -> hardware.intake1.setPower(Tuning_Constant.testing_Rear_Intake_Power)),
                    instant(() -> hardware.blocker.setPosition(0))
            );
        }
        public Command shaking(){
            double heading = follower.getHeading();
            return sequential(
                    turnTo(follower, heading + Math.toRadians(10)),
                    turnTo(follower, heading + Math.toRadians(-10)),
                    turnTo(follower, heading + Math.toRadians(0))

            );
        }

    }
    @Autonomous(name = "紅方遠自動", group ="Far")
    public static class RedFarAuto extends BaseFarAuto{
        @Override
        protected boolean getIsBlue() {
            return false;
        }
    }
}
