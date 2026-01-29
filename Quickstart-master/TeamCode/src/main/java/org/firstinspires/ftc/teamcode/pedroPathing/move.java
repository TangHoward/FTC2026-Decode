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
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.TeleOpHeadingPD;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constants;

    import java.util.function.Supplier;

    public class move{
        @Configurable
        abstract static class BaseTeleOp extends OpMode{

            protected Follower follower;
            protected boolean automatedDrive;
            protected Supplier<PathChain> pathChain;
            protected TelemetryManager telemetryM;
            protected FtcDashboard dashboard = FtcDashboard.getInstance();
            protected boolean slowMode = false;

            protected Hardware hardware = new Hardware();

            protected TeleOpHeadingPD teleOpHeadingPD;

            protected All_Calculation all_calculation;
            protected double previousServoCalculatePosition = -1;
            protected final double smoothingFactor = 1;
            protected double targetHeading = 0;
            protected abstract Pose getStartingPose();
            protected abstract Pose getAutomatedPathTargetPose();
            protected abstract Pose getAutoAimTargetPose();
            protected abstract boolean getIsBlue();


            @Override
            public void init() {

                hardware.init(hardwareMap);

                final Pose startingPose = getStartingPose();
                follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(Configurable_Constants.botPose == null ? startingPose : Configurable_Constants.botPose);
                follower.update();

                telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
                Drawing.init();


                double[] targetXYZ= {getAutoAimTargetPose().getX(),getAutoAimTargetPose().getY(),45 };
                double[] obstacle = {getIsBlue() ? 25 : 119,144, getIsBlue() ? 5 : 139,120 , 45};
                teleOpHeadingPD = new TeleOpHeadingPD(Configurable_Constants.heading_kp, Configurable_Constants.heading_kd, targetHeading);
                all_calculation = new All_Calculation(targetXYZ,follower, obstacle);

            }

            @Override
            public void start() {
                follower.startTeleopDrive();
            }

            @Override
            public void loop() {

                Pose startingPose = getStartingPose();
                Pose pathTargetPose = getAutomatedPathTargetPose();
                Pose aimTargetPose = getAutoAimTargetPose();

                TelemetryPacket packet = new TelemetryPacket();

                pathChain = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, pathTargetPose)))
                        .setHeadingInterpolation(HeadingInterpolator.facingPoint(aimTargetPose.getX() + InGameTuning.longLunchBallXError, aimTargetPose.getY()))
                        .build();

                Configurable_Constants.botPose = follower.getPose();
                String[] lunchZone ={"near","none","far"};
                int whichzone = 0;
                whichzone = WhichZone();

                follower.update();
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();

                double[] solution = all_calculation.solveShooterRPMAndAngle();

                if (solution[0] > 0) {
                    double rpm = solution[0];
                    double angleRad = solution[1];

                    double rawTargetPosition = all_calculation.calculateServoPosition(angleRad, 180,WhichZone());
                    if (previousServoCalculatePosition == -1) {
                        previousServoCalculatePosition = rawTargetPosition;
                    }

                    double smoothedPosition = previousServoCalculatePosition + smoothingFactor * (rawTargetPosition - previousServoCalculatePosition);
                    if(whichzone ==0 || whichzone ==2){
                        hardware.shooter.setVelocity(rpm / 60 * 28+50);
                        packet.put("shootertargetRPM", rpm);
                    }
                    hardware.angleController.setPosition(smoothedPosition);
                    previousServoCalculatePosition = smoothedPosition;
                }
                telemetryM.update();
                //對於目前位置而做出的舉動
                switch (whichzone) {
                    case 0:
                        hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_nearlunch_KP, 0, Configurable_Constants.shooter_nearlunch_KD, Configurable_Constants.shooter_nearlunch_F);
                        teleOpHeadingPD.setTargetHeading(Math.atan2(getAutoAimTargetPose().getY()-follower.getPose().getY(),
                                getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.nearLunchBallXError)-follower.getPose().getX()));
                        telemetry.addData("TargetPose",getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.nearLunchBallXError));
                        break;
                    case 1:
                        hardware.shooter.setVelocity(4000/60*28+50);
                        packet.put("shootertargetRPM", 4000);
                        break;
                    case 2:
                        hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_longlunch_KP, 0, Configurable_Constants.shooter_longlunch_KD, Configurable_Constants.shooter_longlunch_F);

                        teleOpHeadingPD.setTargetHeading(Math.atan2(getAutoAimTargetPose().getY(),
                                getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.longLunchBallXError)-follower.getPose().getX() ));
                        break;

                }

                double currentTimeSeconds = getRuntime();

                teleOpHeadingPD.setCoefficients(Configurable_Constants.heading_kp, Configurable_Constants.heading_kd);
                /*teleOpHeadingPD.setTargetHeading(Math.atan2(getAutoAimTargetPose().getY()-follower.getPose().getY(),
                        getAutoAimTargetPose().getX()-follower.getPose().getX()));*/
                if (!automatedDrive) {

                    if (!slowMode) {
                        // 蕭濬鑫 false --> true
                        follower.setTeleOpDrive(
                                -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1),
                                -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1),
                                !gamepad1.right_bumper ?  -gamepad1.right_stick_x
                                        : teleOpHeadingPD.calculateTurnPower(follower.getHeading(),currentTimeSeconds),
                                false // Robot Centric
                        );
                    }
                    else{
                        follower.setTeleOpDrive(
                                -gamepad1.left_stick_y * Configurable_Constants.slow_mode_mutiplier * (getIsBlue() ? -1 : 1),
                                -gamepad1.left_stick_x * Configurable_Constants.slow_mode_mutiplier * (getIsBlue() ? -1 : 1),
                                !gamepad1.right_bumper ?  -gamepad1.right_stick_x
                                        : teleOpHeadingPD.calculateTurnPower(follower.getHeading(),currentTimeSeconds)
                                        * Configurable_Constants.slow_mode_mutiplier,
                                false
                                // Robot Centric
                        );
                    }
                    if(gamepad1.b){
                        hardware.intake.setPower(1);
                    } else if (gamepad1.a) {
                        hardware.intake.setPower(0.3);
                    }else {
                        hardware.intake.setPower(0);
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
                    if (gamepad2.aWasPressed() && true) {
                        follower.followPath(pathChain.get());
                        automatedDrive = true;
                    }


                    //Slow Mode
                    if (gamepad2.leftBumperWasPressed()) {
                        slowMode = !slowMode;
                    }
                }else {
                    if (gamepad2.bWasPressed() || !follower.isBusy()) {
                        follower.startTeleopDrive();
                        automatedDrive = false;
                    }
                }
                telemetryM.debug("position", follower.getPose());
                telemetryM.debug("velocity", follower.getVelocity());
                telemetryM.debug("automatedDrive", automatedDrive);

                telemetry.addData("伺服馬達位置", hardware.angleController.getPosition());
                telemetry.addData("伺服馬達設定位置",previousServoCalculatePosition);
                telemetry.addData("射擊輪速度RPM",hardware.shooter.getVelocity()*60/28);
                telemetry.addData("follower狀態", follower.isBusy() ? "true" : "false");
                telemetry.addData("是否在射擊範圍內",follower.getPose().getX() >= 48 &&
                                                          follower.getPose().getX() <= 96 &&
                                                          follower.getPose().getY() >= 0 &&
                                                          follower.getPose().getY() <= 48 ? "true" : "false");
                telemetry.addData("射擊範圍",lunchZone[whichzone]);
                telemetry.addData("計算角度",solution[1]);
                telemetry.addData("計算RPM",solution[0]);

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
        @TeleOp(name = "紅方操控程式",group = "RED")
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
        @TeleOp(name = "藍方操控程式",group = "BLUE")
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
