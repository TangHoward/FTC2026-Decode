package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.All_Calculation;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.InGameTuning;


public class Auto {
        abstract static class BaseFarAuto extends OpMode {

        //protected int pattern_number = 0;
        //變數設定
        private TelemetryManager telemetryM;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState = 0;

        private Hardware hardware = new Hardware();

        protected abstract boolean getIsBlue();

        protected abstract boolean Enable_1st();
        protected abstract boolean Enable_2nd();
        protected abstract boolean Enable_3rd();
        protected abstract boolean Enable_Hide();
        protected abstract boolean Enable_gate();
        private boolean done_1st = false, done_2nd= false, done_3rd = false;
        private All_Calculation all_calculation;

        protected abstract Pose getAutoAimTargetPose();

        // 每個 "點(位置)" 的宣告
        private Pose shootTargetPose;
        private final Pose startingPose = new Pose(
                Math.abs(96 - (getIsBlue() ? 144 : 0)),
                8,
                Math.toRadians(Math.abs(90 - (getIsBlue() ? 180 : 0)))
        );

        private final Pose shootPose = new Pose(
                Math.abs(88 - (getIsBlue() ? 144 : 0)),
                15
        );
        private final Pose humanElementsPose = new Pose(
                Math.abs(129 - (getIsBlue() ? 144 : 0)),
                11,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose firstElement = new Pose(
                Math.abs(130.5 - (getIsBlue() ? 144 : 0)),
                35,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose firstElementControlPoint = new Pose(
                Math.abs(88 - (getIsBlue() ? 144 : 0)),
                45
        );

        private final Pose SecondElement = new Pose(
                Math.abs(130.5 - (getIsBlue() ? 144 : 0)),
                60,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose SecondElementControlPoint = new Pose(
                Math.abs(88 - (getIsBlue() ? 144 : 0)),
                68
        );
        private final Pose thirdElement = new Pose(
                Math.abs(125 - (getIsBlue() ? 144 : 0)),
                84,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose thirdElementControlPoint = new Pose(
                Math.abs(88 - (getIsBlue() ? 144 : 0)),
                92
        );

        private final Pose gateDidPush = new Pose(
                Math.abs(124.6 - (getIsBlue() ? 144 : 0)),
                70,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose gateNoPush = new Pose(
                Math.abs(115 - (getIsBlue() ? 144 : 0)),
                70,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        //this GateControlPoint is for shootPose to gate
        private final Pose goToGateControlPoint = new Pose(
                Math.abs(110 - (getIsBlue() ? 144 : 0)),
                51.4
        );
        private final Pose gateGoToShootControlPoint = new Pose(
                Math.abs(75 - (getIsBlue() ? 144 : 0)),
                67
        );
        private final Pose finishPoint = new Pose(
                Math.abs(88 -(getIsBlue()? 144:0)),
                48
                ,Math.toRadians(Math.abs(180 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose HidePoint = new Pose(
                Math.abs(120 -(getIsBlue()? 144:0)),
                12
                ,Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );


        // 每個 "路徑" 的宣告
        private Path start;
        private PathChain pick1stElement, gotoShootPose,
                pick2ndElement,gotoShootPoseBending2nd,
                pick3rdElement,gotoShootPoseBending3rd,
                goToShootPoseGateBending,
                goToGateDidPushStraight, goToGateDidPushBending,goToGateNoPushStraight, goToGateNoPushBending,
                goToFinishPoint, goToHidePoint;
        // 給每個路徑工作的副程式
        private void buildPath() {
            // 靜態宣告(因為起點都是設定的)
            start = new Path(new BezierLine(startingPose, shootPose));
            start.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0.0, 0.1,
                            HeadingInterpolator.constant(startingPose.getHeading())
                    ),

                    new HeadingInterpolator.PiecewiseNode(
                            0.1, 1.0,
                            HeadingInterpolator.facingPoint(shootTargetPose)
                    )
            ));
            /*
            這個會把朝向的路徑分段 全部一個路徑是100% 所以這個把它分成了兩段
            有一段是0%~10% 的路徑都會跟startingPose 的朝向一樣
            有一段則是10%~100% 的路逕會面朝射擊目標位置(shootTargetPose)
            * new HeadingInterpolator.PiecewiseNode(
                            0.0, 0.1,
                            HeadingInterpolator.constant(startingPose.getHeading())
                    ),

                    new HeadingInterpolator.PiecewiseNode(
                            0.1, 1.0,
                            HeadingInterpolator.facingPoint(shootTargetPose)
                    )
                    */

            // 動態宣告(通過機器位置 和控制點、目標點 去做路徑)
            pick1stElement =follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,firstElementControlPoint,firstElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,firstElement.getHeading(),0.5))
                    .build();
            pick2ndElement = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, SecondElementControlPoint, SecondElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, SecondElement.getHeading(), 0.5))
                    .build();
            pick3rdElement = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, thirdElementControlPoint, thirdElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, thirdElement.getHeading(), 0.5))
                    .build();

            gotoShootPoseBending3rd = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,thirdElementControlPoint,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootTargetPose))
                    .build();

            gotoShootPoseBending2nd = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,SecondElementControlPoint,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0.0, 0.3,
                                    HeadingInterpolator.constant(gateDidPush.getHeading())
                            ),

                            new HeadingInterpolator.PiecewiseNode(
                                    0.3, 1.0,
                                    HeadingInterpolator.facingPoint(shootTargetPose)
                            )
                    ))
                    .build();

            goToShootPoseGateBending = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, gateGoToShootControlPoint,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0.0, 0.3,
                                    HeadingInterpolator.constant(gateDidPush.getHeading())
                            ),

                            new HeadingInterpolator.PiecewiseNode(
                                    0.3, 1.0,
                                    HeadingInterpolator.facingPoint(shootTargetPose)
                            )
                    ))
                    .build();

            gotoShootPose = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootTargetPose))
                    .build();

            goToGateDidPushBending = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, goToGateControlPoint,gateDidPush))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateDidPush.getHeading(), 0))
                    .build();

            goToFinishPoint = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, finishPoint))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, finishPoint.getHeading(),0.6))
                    .build();
            goToHidePoint = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, HidePoint))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, HidePoint.getHeading(),0.6))
                    .build();
            goToGateNoPushStraight = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateNoPush))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateNoPush.getHeading(),0.6))
                    .build();
        }

        private void autonomousPathUpdate() {
            // 控制機器目前狀態(做到哪了)
            switch (pathState) {
                case 0://啟動射擊且開始移動
                    shooting(true);
                    intaking(true);
                    hardware.angleController.setPosition(Configurable_Constants.angleControlLong);
                    follower.followPath(start, true);
                    setPathState(1);
                    break;
                case 1://等待
                    //等待直到機器到點 和 射擊輪速度到要求範圍
                    if(!follower.isBusy()&& Configurable_Constants.shooterLongRangeSpeed / 60 * 28 -10 > hardware.shooter.getVelocity()  / 60 * 28) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    //開始射擊
                    transforming(true);
                    //等待某個秒數後到state 100 進行判斷
                    waitUntil(2.7,100);
                    break;
                // 判斷是否已經做過某件事 或是 是否需要做某件事
                case 100:
                    if(Enable_2nd() && !done_2nd){
                        setPathState(201);
                    } else if (Enable_1st() && !done_1st) {
                        setPathState(101);
                    } else if (Enable_3rd() && !done_3rd) {
                        setPathState(301);
                    }else setPathState(3);
                    break;
                case 201:
                    transforming(false);
                    follower.followPath(pick2ndElement,true);
                    intaking(true);
                    //判斷是否需要推門
                    setPathState(Enable_gate() ? 202 : 204);
                    break;
                case 202:
                    if(!follower.isBusy()) {
                        follower.followPath(goToGateDidPushBending,true);
                        setPathState(203);
                    }
                    break;
                case 203:
                    if(!follower.isBusy()){
                        follower.followPath(goToShootPoseGateBending);
                        setPathState(205);
                    }
                    break;
                case 204:
                    if(!follower.isBusy()){
                        follower.followPath(gotoShootPoseBending2nd);
                        setPathState(205);
                    }
                    break;
                case 205:
                    if(!follower.isBusy()&& Configurable_Constants.shooterLongRangeSpeed / 60 * 28 -10 > hardware.shooter.getVelocity()  / 60 * 28) {
                        setPathState(206);
                    }
                    break;
                case 206:
                    transforming(true);
                    done_2nd = true;
                    waitUntil(2.7,100);
                    break;

                case 101:
                    transforming(false);
                    follower.followPath(pick1stElement,true);
                    intaking(true);
                    setPathState(102);
                    break;
                case 102:
                    if(!follower.isBusy()) {
                        follower.followPath(gotoShootPose,true);
                        setPathState(103);
                    }
                    break;
                case 103:
                    if(!follower.isBusy()&& Configurable_Constants.shooterLongRangeSpeed / 60 * 28 -10 > hardware.shooter.getVelocity()  / 60 * 28){
                        setPathState(104);
                    }
                    break;
                case 104:
                    transforming(true);
                    done_1st = true;
                    waitUntil(2.7,100);
                    break;

                case 301:
                    transforming(false);
                    follower.followPath(pick3rdElement);
                    intaking(true);
                    setPathState(302);
                    break;
                case 302:
                    if(!follower.isBusy()){
                        follower.followPath(gotoShootPoseBending3rd);
                        setPathState(303);
                    }
                    break;
                case 303:
                    if(!follower.isBusy()&& Configurable_Constants.shooterLongRangeSpeed / 60 * 28 -50 > hardware.shooter.getVelocity()  / 60 * 28){
                        setPathState(304);
                    }
                    break;
                case 304:
                    transforming(true);
                    done_3rd = true;
                    waitUntil(2.7,100);
                    break;

                case 3:
                    transforming(false);
                    shooting(false);
                    follower.followPath(Enable_Hide() ? goToHidePoint : goToGateNoPushStraight);
                    setPathState(4);
                    break;
                case 4:
                    break;
            }
        }
        // 設定機器狀態 (會重製某些時鐘)
        private void setPathState (int pState){
            pathState = pState;
            pathTimer.resetTimer();
            actionTimer.resetTimer();
        }
        // 當機器開始時的迴圈 一直更新東西 和顯示數據
        @Override
        public void loop () {
            follower.update();
            autonomousPathUpdate();
            Configurable_Constants.botPose = follower.getPose();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
            packet.put("shooterTargetRPM", Configurable_Constants.shooterLongRangeSpeed);

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower狀態", follower.isBusy() ? "true" : "false");
            telemetry.addData("timer",actionTimer.getElapsedTimeSeconds());
            telemetry.update();
            move.dashboard.sendTelemetryPacket(packet);
        }
        // 機器一開始做準備的地方(機器還不能動!!)
        @Override
        public void init () {
            shootTargetPose= new Pose(
                    Math.abs (getAutoAimTargetPose().getX() -(getIsBlue() ? 144 : 0)),
                    getAutoAimTargetPose().getY()
            );

            hardware.init(hardwareMap);
            hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_longlunch_KP, 0, Configurable_Constants.shooter_longlunch_KD, Configurable_Constants.shooter_longlunch_F);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
            packet.put("shooterTargetRPM", Configurable_Constants.shooterLongRangeSpeed);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            actionTimer = new Timer();
            follower = Constants.createFollower(hardwareMap);
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            buildPath();
            follower.setStartingPose(startingPose);

            telemetryM.debug("A1. 起點到射擊點", start);


            double[] targetXYZ= {Math.abs(getAutoAimTargetPose().getX() - (getIsBlue() ? 144 : 0))
                    ,getAutoAimTargetPose().getY(),45};
            double[] obstacle = {getIsBlue() ? 25 : 119,144, getIsBlue() ? 5 : 139,120 , 45};

            all_calculation = new All_Calculation(targetXYZ,follower, obstacle);
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
            telemetryM.update();
            move.dashboard.sendTelemetryPacket(packet);
        }
        // 機器開始時會做的第一件事情
        @Override
        public void start () {
            opmodeTimer.resetTimer();
            setPathState(Configurable_Constants.autostartstate);
            intaking(false);
        }
        // 撿球開關的副程式
        private void intaking ( boolean onOff){
            double power = onOff ? 1 : 0;
            hardware.intake.setPower(power);
            hardware.transferServo0.setPower(power);
            hardware.transferServo1.setPower(power);
        }
        // 傳輸到射擊輪開關的副程式
        private void transforming(boolean onOff){
            double power = onOff ? 1 : 0;
            hardware.intake.setPower(power*0.3);
            hardware.transferServo0.setPower(power);
            hardware.transferServo1.setPower(power);
            hardware.transferServo2.setPower(power*0.6);
        }
        // 射擊輪開關的副程式
        private void shooting(boolean onOff){
            double[] solution = all_calculation.solveShooterRPMAndAngle();
            if(onOff){
                hardware.shooter.setVelocity(Configurable_Constants.shooterLongRangeSpeed / 60 * 28);
            }else{
                hardware.shooter.setVelocity(0);
            }
        }
        // 等待的副程式
        private void waitUntil(double sec, int nextstate){
                if(actionTimer.getElapsedTimeSeconds() >= sec)setPathState(nextstate);
            }
}

    @Autonomous(name = "紅方遠自動程式", group = "RED")
    public static class RedFarAutonomous extends BaseFarAuto {
        @Override //第一排的球
        protected boolean Enable_1st(){return false;}
        @Override //第二排的球
        protected boolean Enable_2nd(){
            return false;
        }
        @Override //第三排的球
        protected boolean Enable_3rd(){
            return false;
        }
        @Override //要不要躲
        protected boolean Enable_Hide(){
            return true;
        }
        @Override //要不要去推牆
        protected boolean Enable_gate(){
            return false;
        }
        @Override
        protected boolean getIsBlue() {
            return false;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Configurable_Constants.target_X+ InGameTuning.nearLunchBallXError,144);
        }

    }
    @Autonomous(name = "藍方遠自動程式", group = "BLUE")
    public static class BlueFarAutonomous extends BaseFarAuto {
        @Override //第一排球
        protected boolean Enable_1st(){
            return false;
        }
        @Override //第二排球
        protected boolean Enable_2nd(){
            return false;
        }
        @Override //第三排球
        protected boolean Enable_3rd(){
            return false;
        }
        @Override //要不要躲
        protected boolean Enable_Hide(){
            return true;
        }
        @Override //要不要推門
        protected boolean Enable_gate(){
            return false;
        }
        @Override //要不要躲
        protected boolean getIsBlue() {
            return true;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Configurable_Constants.target_X+ InGameTuning.nearLunchBallXError,144);
        }

    }


    abstract static class BasCloseAuto extends OpMode {
        private TelemetryManager telemetryM;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState = 21;

        private Hardware hardware = new Hardware();

        protected abstract boolean getIsBlue();
        protected abstract boolean Enable_1st();
        protected abstract boolean Enable_2nd();
        protected abstract boolean Enable_3rd();
        protected abstract boolean Enable_Hide();
        protected abstract boolean Enable_gate();
        private boolean done_1st = false, done_2nd= false, done_3rd = false;
        private All_Calculation all_calculation;

        protected abstract Pose getAutoAimTargetPose();

        private   Pose shootTargetPose;


        private final Pose shootPose = new Pose(
                Math.abs(96 - (getIsBlue() ? 144 : 0)),
                86
        );
        private final Pose startingPose = new Pose(
                Math.abs(118.3 - (getIsBlue() ? 144 : 0)),
                127.5,
                Math.toRadians(Math.abs(40.3 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose humanElementsPose = new Pose(
                Math.abs(129 - (getIsBlue() ? 144 : 0)),
                11,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );

        private final Pose firstElement = new Pose(
                Math.abs(130.5 - (getIsBlue() ? 144 : 0)),
                35,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose firstElementControlPoint = new Pose(
                Math.abs(80 - (getIsBlue() ? 144 : 0)),
                30
        );
        private final Pose firstElementGoToShootPose = new Pose(
                Math.abs(110 - (getIsBlue() ? 144 : 0)),
                35
        );

        private final Pose SecondElement = new Pose(
                Math.abs(130.5 - (getIsBlue() ? 144 : 0)),
                60,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose SecondElementControlPoint = new Pose(
                Math.abs(80 - (getIsBlue() ? 144 : 0)),
                57
        );
        private final Pose thirdElement = new Pose(
                Math.abs(125 - (getIsBlue() ? 144 : 0)),
                84,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose thirdElementControlPoint = new Pose(
                Math.abs(120 - (getIsBlue() ? 144 : 0)),
                81
        );

        private final Pose gateDidPush = new Pose(
                Math.abs(124.6 - (getIsBlue() ? 144 : 0)),
                70,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose gateNoPush = new Pose(
                Math.abs(115 - (getIsBlue() ? 144 : 0)),
                70,
                Math.toRadians(Math.abs(0 - (getIsBlue() ? 180 : 0)))
        );
        //this GateControlPoint is for shootPose to gate
        private final Pose goToGateControlPoint = new Pose(
                Math.abs(110 - (getIsBlue() ? 144 : 0)),
                51.4
        );
        private final Pose gateGoToShootControlPoint = new Pose(
                Math.abs(75 - (getIsBlue() ? 144 : 0)),
                67
        );
        private final Pose finishPoint = new Pose(
                Math.abs(88 -(getIsBlue()? 144:0)),
                48
                ,Math.toRadians(Math.abs(180 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose hidePoint = new Pose(
                Math.abs(123 -(getIsBlue()? 144:0)),
                103
                ,Math.toRadians(Math.abs(180 - (getIsBlue() ? 180 : 0)))
        );
        private final Pose hideControlPoint = new Pose(
                Math.abs(102 -(getIsBlue()? 144:0)),
                111
        );

        private Path start;
        private PathChain pick1stElement, gotoShootPose,goToShootPoseGateBending,
                pick2ndElement,gotoShootPoseBending2nd,
                pick3rdElement,gotoShootPoseBending1st,
                goToGateDidPushBending,goToGateNoPushStraight
                ,goToFinishPoint,goToHidePoint;

        private void buildPath() {
            start = new Path(new BezierLine(startingPose, shootPose));
            start.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0.0, 0.1,
                            HeadingInterpolator.constant(startingPose.getHeading())
                    ),

                    new HeadingInterpolator.PiecewiseNode(
                            0.1, 1.0,
                            HeadingInterpolator.facingPoint(shootTargetPose)
                    )
            ));



            pick1stElement =follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,firstElementControlPoint,firstElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,firstElement.getHeading(),0.5))
                    .setTranslationalConstraint(0.5)
                    .build();
            pick2ndElement = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, SecondElementControlPoint, SecondElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, SecondElement.getHeading(), 0.5))
                    .build();
            pick3rdElement = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, thirdElementControlPoint, thirdElement))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, thirdElement.getHeading(), 0))
                    .build();

            gotoShootPoseBending1st = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,firstElementGoToShootPose,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootTargetPose))
                    .setHeadingConstraint(0.4)
                    .build();
            gotoShootPoseBending2nd = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose,SecondElementControlPoint,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0.0, 0.3,
                                    HeadingInterpolator.constant(gateDidPush.getHeading())
                            ),

                            new HeadingInterpolator.PiecewiseNode(
                                    0.3, 1.0,
                                    HeadingInterpolator.facingPoint(shootTargetPose)
                            )
                    ))
                    .build();

            goToShootPoseGateBending = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, gateGoToShootControlPoint,shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0.0, 0.3,
                                    HeadingInterpolator.constant(gateDidPush.getHeading())
                            ),

                            new HeadingInterpolator.PiecewiseNode(
                                    0.3, 1.0,
                                    HeadingInterpolator.facingPoint(shootTargetPose)
                            )
                    ))
                    .build();

            gotoShootPose = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, shootPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootTargetPose))
                    .build();

            goToGateDidPushBending = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, goToGateControlPoint,gateDidPush))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateDidPush.getHeading(), 0))
                    .build();
            goToGateNoPushStraight = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, gateNoPush))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateNoPush.getHeading(),0))
                    .build();
            goToFinishPoint = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose,finishPoint))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,finishPoint.getHeading(),0.6))
                    .build();

            goToHidePoint = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, hideControlPoint,hidePoint))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, hidePoint.getHeading(),1))
                    .build();
        }

        private void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    shooting(true);
                    hardware.angleController.setPosition(Configurable_Constants.angleControlNear);
                    follower.followPath(start, true);
                    setPathState(1);
                    break;
                case 1:
                    if (!follower.isBusy() && Configurable_Constants.shooterNearRangeSpeed / 60 * 28 -50 > hardware.shooter.getVelocity()  / 60 * 28) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    transforming(true);
                    waitUntil(3.2, 100);
                    break;
                case 100:
                    if(Enable_2nd() && !done_2nd){
                        setPathState(201);
                    } else if (Enable_3rd() && !done_3rd) {
                        setPathState(301);
                    } else if (Enable_1st() && ! done_1st) {
                        setPathState(101);
                    }else setPathState(3);
                    break;

                case 301:
                    transforming(false);
                    follower.followPath(pick3rdElement,true);
                    intaking(true);
                    setPathState(302);
                    break;
                case 302:
                    if(!follower.isBusy()) {
                        follower.followPath(gotoShootPose,true);
                        setPathState(303);
                    }
                    break;
                case 303:
                    if(!follower.isBusy() && Configurable_Constants.shooterNearRangeSpeed / 60 * 28 -50 > hardware.shooter.getVelocity()  / 60 * 28){
                        setPathState(304);
                    }
                    break;
                case 304:
                    transforming(true);
                    done_3rd = true;
                    waitUntil(3.2,100);
                    break;

                case 201:
                    transforming(false);
                    follower.followPath(pick2ndElement,true);
                    intaking(true);
                    setPathState(202);
                    break;
                case 202:
                    if(!follower.isBusy()) {
                        if(Enable_gate()) {
                            follower.followPath(goToGateDidPushBending, true);
                            setPathState(203);
                        }else {
                            follower.followPath(gotoShootPoseBending2nd, true);
                            setPathState(204);
                        }
                    }
                    break;
                case 203:
                    if(!follower.isBusy()){
                        follower.followPath(goToShootPoseGateBending, true);
                        setPathState(204);
                    }
                    break;
                case 204:
                    if(!follower.isBusy()&& Configurable_Constants.shooterNearRangeSpeed / 60 * 28 -50 > hardware.shooter.getVelocity()  / 60 * 28){
                        setPathState(205);
                    }
                    break;
                case 205:
                    transforming(true);
                    done_2nd = true;
                    waitUntil(3.2,100);
                    break;


                case 101:
                    transforming(false);
                    follower.followPath(pick1stElement,true);
                    intaking(true);
                    setPathState(102);
                    break;
                case 102:
                    if(!follower.isBusy()) {
                        follower.followPath(gotoShootPoseBending1st, true);
                        setPathState(103);
                    }
                    break;
                case 103:
                    if(!follower.isBusy() && Configurable_Constants.shooterNearRangeSpeed / 60 * 28 -50 > hardware.shooter.getVelocity()  / 60 * 28){
                        setPathState(104);
                    }
                    break;
                case 104:
                    transforming(true);
                    done_1st = true;
                    waitUntil(2.9,100);
                    break;


                case 3:
                    transforming(false);
                    shooting(false);
                    follower.followPath(Enable_Hide()? goToHidePoint:goToGateNoPushStraight,true);
                    setPathState(4);
                    break;
                case 4:
                    if(!follower.isBusy()){
                    }
                    break;
            }
        }
        private void setPathState (int pState){
            pathState = pState;
            pathTimer.resetTimer();
            actionTimer.resetTimer();
        }

        @Override
        public void loop () {
            follower.update();
            autonomousPathUpdate();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
            packet.put("shooterTargetRPM", Configurable_Constants.shooterNearRangeSpeed);
            Configurable_Constants.botPose = follower.getPose();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower狀態", follower.isBusy() ? "true" : "false");
            telemetry.addData("timer",actionTimer.getElapsedTimeSeconds());

            move.dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        @Override
        public void init () {


            hardware.init(hardwareMap);
            hardware.shooter.setVelocityPIDFCoefficients(Configurable_Constants.shooter_nearlunch_KP, 0, Configurable_Constants.shooter_nearlunch_KD, Configurable_Constants.shooter_nearlunch_F);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterRPM", hardware.shooter.getVelocity() * 60 / 28);
            packet.put("shooterTargetRPM", Configurable_Constants.shooterNearRangeSpeed);

            shootTargetPose= new Pose(
                    Math.abs (getAutoAimTargetPose().getX() -(getIsBlue() ? 144 : 0)),
                    getAutoAimTargetPose().getY()
            );

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            actionTimer = new Timer();
            follower = Constants.createFollower(hardwareMap);
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            buildPath();
            follower.setStartingPose(startingPose);

            telemetryM.debug("A1. 起點到射擊點", start);


            double[] targetXYZ= {Math.abs(getAutoAimTargetPose().getX() - (getIsBlue() ? 144 : 0))
                    ,getAutoAimTargetPose().getY(),45};
            double[] obstacle = {getIsBlue() ? 25 : 119,144, getIsBlue() ? 5 : 139,120 , 45};

            all_calculation = new All_Calculation(targetXYZ,follower, obstacle);
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
            telemetryM.update();
            move.dashboard.sendTelemetryPacket(packet);
        }

        @Override
        public void start () {
            opmodeTimer.resetTimer();
            setPathState(Configurable_Constants.autostartstate);
            intaking(false);
        }

        private void intaking ( boolean onOff){
            double power = onOff ? 1 : 0;
            hardware.intake.setPower(power);
            hardware.transferServo0.setPower(power);
            hardware.transferServo1.setPower(power);
        }

        private void transforming(boolean onOff){
            double power = onOff ? 1 : 0;
            hardware.intake.setPower(power*0.3);
            hardware.transferServo0.setPower(power);
            hardware.transferServo1.setPower(power);
            hardware.transferServo2.setPower(power *0.3);
        }
        private void shooting(boolean onOff){
            double[] solution = all_calculation.solveShooterRPMAndAngle();
            if(onOff){
                hardware.shooter.setVelocity(Configurable_Constants.shooterNearRangeSpeed / 60 * 28);
            }else{
                hardware.shooter.setVelocity(0);
            }
        }
        private void waitUntil(double sec, int nextstate){
                if(actionTimer.getElapsedTimeSeconds() >= sec)setPathState(nextstate);
        }

    }
    @Autonomous(name = "紅方近自動程式", group = "RED")
    public static class RedCloseAutonomous extends BasCloseAuto {
        @Override //第一排球
        protected boolean Enable_1st(){
            return true;
        }
        @Override //第二排球
        protected boolean Enable_2nd(){
            return true;
        }
        @Override //第三排球
        protected boolean Enable_3rd(){
            return true;
        }
        @Override //要不要躲
        protected boolean Enable_Hide(){
            return false;
        }
        @Override //要不要推門
        protected boolean Enable_gate(){
            return true;
        }
        @Override
        protected boolean getIsBlue() {
            return false;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Configurable_Constants.target_X+ InGameTuning.nearLunchBallXError,144);
        }

    }
    @Autonomous(name = "藍方近自動程式", group = "BLUE")
    public static class BlueCloseAutonomous extends BasCloseAuto {
        @Override
        protected boolean Enable_1st(){
            return true;
        }
        @Override
        protected boolean Enable_2nd(){
            return true;
        }
        @Override
        protected boolean Enable_3rd(){
            return true;
        }
        @Override
        protected boolean Enable_Hide(){
            return false;
        }
        @Override
        protected boolean Enable_gate(){
            return true;
        }
        @Override
        protected boolean getIsBlue() {
            return true;
        }
        @Override
        protected Pose getAutoAimTargetPose() {
            return new Pose(Configurable_Constants.target_X+ InGameTuning.nearLunchBallXError,144);
        }

    }
}

