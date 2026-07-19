package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

public class Localize{
    abstract static class LocalizeTest extends OpMode {
        public static Follower follower;
        static TelemetryManager telemetryM;
        protected abstract Pose getPose();
        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(getPose());

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            Drawing.init();
        }

        /**
         * This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.
         */
        @Override
        public void init_loop() {
            telemetryM.debug("This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1.");
            telemetryM.update(telemetry);
            follower.update();
            drawOnlyCurrent();
        }

        @Override
        public void start() {
            follower.startTeleopDrive();
            follower.update();
        }

        /**
         * This updates the robot's pose estimate, the simple mecanum drive, and updates the
         * Panels telemetry with the robot's position as well as draws the robot's position.
         */
        @Override
        public void loop() {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            telemetryM.debug("x:" + follower.getPose().getX());
            telemetryM.debug("y:" + follower.getPose().getY());
            telemetryM.debug("heading:" +Math.toDegrees(follower.getPose().getHeading()));
            telemetryM.debug("total heading:" + follower.getTotalHeading());
            telemetryM.update(telemetry);

            draw();
        }
        public static void drawOnlyCurrent() {
            try {
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed " + e);
            }
        }

        public static void draw() {
            Drawing.drawDebug(follower);
        }
    }
//    @TeleOp
    public static class PoseTest extends LocalizeTest{
        @Override
        protected Pose getPose() {
            return new Pose(Tuning_Constant.x_localizeTest_startPose, Tuning_Constant.y_localizeTest_startPose, Math.toRadians(Tuning_Constant.r_localizeTest_startPose));
        }
    }
}
