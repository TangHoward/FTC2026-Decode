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

    public class move{
        public static FtcDashboard dashboard = FtcDashboard.getInstance();

        @Configurable
        abstract static class BaseTeleOp extends OpMode{
            // 變數宣告
            private Follower follower;
            private boolean automatedDrive;
            private Supplier<PathChain> pathChain;
            private TelemetryManager telemetryM;
            private boolean slowMode = false;

            private Hardware hardware = new Hardware();

            private TeleOpHeadingPD_Pose teleOpHeadingPD_Pose;
            private TeleOpHeadingPD_Cam teleOpHeadingPD_cam;

            private All_Calculation all_calculation;
            private double previousServoCalculatePosition = -1;
            private final double smoothingFactor = 1;
            private double targetHeading = 0;
            protected abstract Pose getStartingPose();
            protected abstract Pose getAutomatedPathTargetPose();
            protected abstract Pose getAutoAimTargetPose();
            protected abstract boolean getIsBlue();

            // (初始化)機器一開始會做的事情 (機器不能動!!)
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
                teleOpHeadingPD_Pose = new TeleOpHeadingPD_Pose(Configurable_Constants.heading_kp_Pose, Configurable_Constants.heading_kd_Pose, targetHeading);
                teleOpHeadingPD_cam = new TeleOpHeadingPD_Cam(Configurable_Constants.heading_kp_Cam, Configurable_Constants.heading_kd_Cam, getIsBlue() ? 20: 24, hardware);
                all_calculation = new All_Calculation(targetXYZ,follower, obstacle);

            }

            // 機器開始會做的事情
            @Override
            public void start() {
                follower.startTeleopDrive();
            }
            // 初始化迴圈
            @Override
            public void init_loop(){
                FtcDashboard.getInstance().startCameraStream(hardware.visionPortal, 0);
            }
            // 開始的迴圈(直到結束)
            @Override
            public void loop() {

                Pose startingPose = getStartingPose();
                Pose pathTargetPose = getAutomatedPathTargetPose();
                Pose aimTargetPose = getAutoAimTargetPose();

                TelemetryPacket packet = new TelemetryPacket();

                // 實時派給這個路徑工作
                pathChain = () -> follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, pathTargetPose)))
                        .setHeadingInterpolation(HeadingInterpolator.facingPoint(aimTargetPose.getX() + InGameTuning.longLunchBallXError, aimTargetPose.getY()))
                        .build();
                //判斷機器在哪個區域
                Configurable_Constants.botPose = follower.getPose();
                String[] lunchZone ={"near","none","far"};
                int whichzone = 0;
                whichzone = WhichZone();

                follower.update();
                //把機器在panel上畫出來
                Drawing.drawRobot(follower.getPose());
                Drawing.sendPacket();

                // All_Calculation 的計算值
                double[] solution = all_calculation.solveShooterRPMAndAngle();
                // 依計算後的數值去決定要怎麼做
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
                //對於機器人所在的區域而做出的舉動
                switch (whichzone) {
                    case 0:
                        hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_nearlunch_KP, 0, Configurable_Constants.shooter_nearlunch_KD, Configurable_Constants.shooter_nearlunch_F);
                        teleOpHeadingPD_Pose.setTargetHeading(Math.atan2(getAutoAimTargetPose().getY()-follower.getPose().getY(),
                                getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.nearLunchBallXError)-follower.getPose().getX()));
                        telemetry.addData("TargetPose",getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.nearLunchBallXError));
                        break;
                    case 1:
                        hardware.shooter.setVelocity(4250 /60*28+50);
                        packet.put("shootertargetRPM", 4250);
                        break;
                    case 2:
                        hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_longlunch_KP, 0, Configurable_Constants.shooter_longlunch_KD, Configurable_Constants.shooter_longlunch_F);

                        teleOpHeadingPD_Pose.setTargetHeading(Math.atan2(getAutoAimTargetPose().getY(),
                                getAutoAimTargetPose().getX() + ((getIsBlue() ? 1:-1) * InGameTuning.longLunchBallXError)-follower.getPose().getX() ));
                        break;

                }
                // 因應PID 所需的時間
                double currentTimeSeconds = getRuntime();
                teleOpHeadingPD_Pose.setCoefficients(Configurable_Constants.heading_kp_Pose, Configurable_Constants.heading_kd_Pose);
                teleOpHeadingPD_cam.setCoefficients(Configurable_Constants.heading_kp_Cam,Configurable_Constants.heading_kd_Cam);
                // 因為有自動駕駛 所以這是當沒有使用自動駕駛時會做的事情
                if (!automatedDrive) {
                    // 判斷是否有緩速模式
                    if (!slowMode) {
                        // 機器前後左右或是轉向 數值 0~1
                        // 最後一個false 的意思是是否要用全局去看前後左右還是用機器人的朝向去看前後左右
                        follower.setTeleOpDrive(
                                -gamepad1.left_stick_y * (getIsBlue() ? -1 : 1),
                                -gamepad1.left_stick_x * (getIsBlue() ? -1 : 1),
                                !gamepad1.right_bumper ? (gamepad1.left_bumper && teleOpHeadingPD_cam.foundTarget()? teleOpHeadingPD_cam.calculateTurnPower(currentTimeSeconds)
                                        : -gamepad1.right_stick_x)
                                        : teleOpHeadingPD_Pose.calculateTurnPower(follower.getHeading(),currentTimeSeconds),
                                false // Robot Centric
                        );
                    }
                    else{
                        follower.setTeleOpDrive(
                                -gamepad1.left_stick_y * Configurable_Constants.slow_mode_mutiplier * (getIsBlue() ? -1 : 1),
                                -gamepad1.left_stick_x * Configurable_Constants.slow_mode_mutiplier * (getIsBlue() ? -1 : 1),
                                !gamepad1.right_bumper ?  -gamepad1.right_stick_x
                                        : teleOpHeadingPD_Pose.calculateTurnPower(follower.getHeading(),currentTimeSeconds)
                                        * Configurable_Constants.slow_mode_mutiplier,
                                false
                                // Robot Centric
                        );
                    }


                    packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
                    dashboard.sendTelemetryPacket(packet);


                    // 啟動自動駕駛
                    if (gamepad2.aWasPressed() && true) {
                        follower.followPath(pathChain.get());
                        automatedDrive = true;
                    }



                }else {
                    //如果正在自動駕駛時 會做的事情
                    if (gamepad2.bWasPressed() || !follower.isBusy()) {
                        follower.startTeleopDrive();
                        automatedDrive = false;
                    }
                }
                //無論有沒有自動駕駛都會做的事情
                if(gamepad1.b){
                    hardware.intake.setPower(1);
                } else if (gamepad1.a) {
                    hardware.intake.setPower(0.3);
                }else {
                    hardware.intake.setPower(0);
                }

                hardware.transferServo0.setPower(gamepad1.b || gamepad1.a ? 1 : 0);
                hardware.transferServo1.setPower(gamepad1.b || gamepad1.a ? 1 : 0);
                hardware.transferServo2.setPower(gamepad1.a ? 1 : 0);
                // 即時使用手把調整射擊角度(避免里程計誤差)
                if(gamepad2.dpadUpWasPressed()){
                    if(whichzone == 0){
                        InGameTuning.nearLunchAngleError -= 1;
                    }else {
                        InGameTuning.longLunchAngleError -= 1;
                    }
                } else if (gamepad2.dpadDownWasPressed()) {
                    if(whichzone == 0){
                        InGameTuning.nearLunchAngleError += 1;
                    }else {
                        InGameTuning.longLunchAngleError += 1;
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
                telemetry.addData("是否有看到april tag", teleOpHeadingPD_cam.foundTarget() ? "有": "沒有" );

            }
            //判斷機器位置
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
