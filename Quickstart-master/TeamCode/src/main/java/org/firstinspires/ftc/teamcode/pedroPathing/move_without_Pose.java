package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.All_Calculation;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.InGameTuning;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.TeleOpHeadingPD_Cam;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.TeleOpHeadingPD_Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constants;

import java.util.function.Supplier;

public class move_without_Pose{
    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    @Configurable
    abstract static class BaseTeleOp extends OpMode{
        protected Follower follower;
        protected TelemetryManager telemetryM;
        protected boolean slowMode = false;

        protected Hardware hardware = new Hardware();
        protected TeleOpHeadingPD_Cam teleOpHeadingPD_cam;
        protected abstract Pose getStartingPose();
        protected abstract Pose getAutomatedPathTargetPose();
        protected abstract Pose getAutoAimTargetPose();
        protected abstract boolean getIsBlue();


        @Override
        public void init() {

            hardware.init(hardwareMap);

            final Pose startingPose = getStartingPose();

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            Drawing.init();


            double[] targetXYZ= {getAutoAimTargetPose().getX(),getAutoAimTargetPose().getY(),45 };
            double[] obstacle = {getIsBlue() ? 25 : 119,144, getIsBlue() ? 5 : 139,120 , 45};
            teleOpHeadingPD_cam = new TeleOpHeadingPD_Cam(Configurable_Constants.heading_kp_Cam, Configurable_Constants.heading_kd_Cam, getIsBlue() ? 20: 24, hardware);

        }

        @Override
        public void start() {
            follower.startTeleopDrive();
        }
        @Override
        public void init_loop(){
            FtcDashboard.getInstance().startCameraStream(hardware.visionPortal, 0);
        }

        @Override
        public void loop() {

            Pose startingPose = getStartingPose();
            Pose pathTargetPose = getAutomatedPathTargetPose();
            Pose aimTargetPose = getAutoAimTargetPose();

            TelemetryPacket packet = new TelemetryPacket();


            Configurable_Constants.botPose = follower.getPose();
            String[] lunchZone ={"near","none","far"};
            int whichzone = 0;
            whichzone = WhichZone();

            follower.update();
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();

            telemetryM.update();

            double currentTimeSeconds = getRuntime();

            teleOpHeadingPD_cam.setCoefficients(Configurable_Constants.heading_kp_Cam,Configurable_Constants.heading_kd_Cam);
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1),
                    -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1),
                    gamepad1.right_bumper && teleOpHeadingPD_cam.foundTarget()? teleOpHeadingPD_cam.calculateTurnPower(currentTimeSeconds)
                    : -gamepad1.right_stick_x,
                    false // Robot Centric
            );
            if(gamepad1.b){
                hardware.intake.setPower(1);
            } else if (gamepad1.a) {
                hardware.intake.setPower(0.3);
            }else {
                hardware.intake.setPower(0);
            }
            if(gamepad1.y){
                hardware.shooter.setVelocity(4400/60*28);
                hardware.angleController.setPosition(0.56);
            } else if (gamepad1.x) {
                hardware.shooter.setVelocity(4400 /60*28);
                hardware.angleController.setPosition(0.5);
            }
            packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
            dashboard.sendTelemetryPacket(packet);

            hardware.transferServo0.setPower(gamepad1.b || gamepad1.a ? 1 : 0);
            hardware.transferServo1.setPower(gamepad1.b || gamepad1.a ? 1 : 0);
            hardware.transferServo2.setPower(gamepad1.a ? 1 : 0);
            /*hardware.angleController.setPosition(gamepad1.dpadUpWasPressed() && hardware.angleController.getPosition() < 1?
                           hardware.angleController.getPosition() +0.05 : hardware.angleController.getPosition());
                    hardware.angleController.setPosition(gamepad1.dpadDownWasPressed() && hardware.angleController.getPosition() > 0?
                            hardware.angleController.getPosition() -0.05 : hardware.angleController.getPosition());*/
                //Automated PathFollowing


                //Slow Mode
            if (gamepad2.leftBumperWasPressed()) {
                slowMode = !slowMode;
            }
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());

            telemetry.addData("伺服馬達位置", hardware.angleController.getPosition());
            telemetry.addData("射擊輪速度RPM",hardware.shooter.getVelocity()*60/28);
            telemetry.addData("follower狀態", follower.isBusy() ? "true" : "false");
            telemetry.addData("是否在射擊範圍內",follower.getPose().getX() >= 48 &&
                    follower.getPose().getX() <= 96 &&
                    follower.getPose().getY() >= 0 &&
                    follower.getPose().getY() <= 48 ? "true" : "false");
            telemetry.addData("射擊範圍",lunchZone[whichzone]);
            telemetry.addData("是否有看到april tag", teleOpHeadingPD_cam.foundTarget() ? "有": "沒有" );

        }
        public int WhichZone(){
            if(follower.getPose().getX() >= 48 &&
                    follower.getPose().getX() <= 96 &&
                    follower.getPose().getY() >= 0 &&
                    follower.getPose().getY() <= 48)
            {
                return 2;
            } else if (follower.getPose().getX() >= 0 &&
                    follower.getPose().getX() <= 144 &&
                    follower.getPose().getY() >= 72 &&
                    follower.getPose().getY() <= 144)
            {
                return 0;
            }
            return 1;
        }
    }
    @TeleOp(name = "紅方操控程式 沒有里程計",group = "RED")
    public static class RedTeleOp extends BaseTeleOp{
        @Override
        protected Pose getStartingPose() {
            return new Pose(72,72,Math.toRadians(90));
        }
        @Override
        protected Pose getAutomatedPathTargetPose() {
            return new Pose(96,96) ;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Configurable_Constants.target_X,144);
        }

        @Override
        protected boolean getIsBlue() {
            return false;
        }
    }
    @TeleOp(name = "藍方操控程式 沒有里程計",group = "BLUE")
    public static class BlueTeleOp extends BaseTeleOp{
        @Override
        protected Pose getStartingPose() {
            return new Pose(72,72, Math.toRadians(90));
        }
        @Override
        protected Pose getAutomatedPathTargetPose() {
            return new Pose(52,15) ;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Math.abs(Configurable_Constants.target_X -144),144);
        }

        @Override
        protected boolean getIsBlue() {
            return true;
        }
    }
}
