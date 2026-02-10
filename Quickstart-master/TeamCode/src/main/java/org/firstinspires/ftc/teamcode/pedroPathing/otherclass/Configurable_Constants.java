package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Configurable_Constants {
    // 數值管理的地方 連接機器後打開http://192.168.43.1:8001/ 可以做到及時調整這些數值

    // 緩速的數值 1~0(幾乎用不到)
    public static double slow_mode_mutiplier = 0.5;

    // 里程計瞄準的PD
    public static double heading_kp_Pose = 1;
    public static double heading_kd_Pose = 0.05;
    // 鏡頭瞄準的PD
    public static double heading_kp_Cam = 0.0005;
    public static double heading_kd_Cam = 0.0;
    // 是否要自動射擊(沒使用)
    public static boolean autoshoot = false;
    // 射擊輪 遠方PIDF
    public static double shooter_longlunch_F = 17;
    public static double shooter_longlunch_KP = 3.2;

    public static double shooter_longlunch_KD = 0;

    // 射擊輪 近方PIDF
    public static double shooter_nearlunch_F = 13.6;
    public static double shooter_nearlunch_KP = 1.8;
    public static double shooter_nearlunch_KD = 0;

    // 機器瞄準目標(透過里程計瞄準)
    public static double target_X = 142.0;
    public static double target_Y = 142.0;
    // 自動開始時 從哪個階段開始
    public static int autostartstate =0;
    // 機器位置 (無法實時更改)
    public static Pose botPose = null;
    // 自動階段的射擊輪速度(RPM) 遠的、近的
    public static int shooterLongRangeSpeed = 5400, shooterNearRangeSpeed = 3400/*4500*/;
    // // 自動階段的控制角度伺服馬達位置 遠的、近的
    public static double angleControlLong = 0.45 , angleControlNear = 0/*0.52*/;


}
