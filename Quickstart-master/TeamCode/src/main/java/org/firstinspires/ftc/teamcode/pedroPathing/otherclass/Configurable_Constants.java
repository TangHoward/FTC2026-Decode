package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Configurable_Constants {
    // 駕駛參數
    public static double slow_mode_mutiplier = 0.5;

    // PD 控制器參數 (即時調校)
    public static double heading_kp_Pose = 1;
    public static double heading_kd_Pose = 0.05;

    public static double heading_kp_Cam = 0.0005;
    public static double heading_kd_Cam = 0.0;

    public static boolean autoshoot = false;

    public static double shooter_longlunch_F = 17;
    public static double shooter_longlunch_KP = 3.2;

    public static double shooter_longlunch_KD = 0;


    public static double shooter_nearlunch_F = 13.6;
    public static double shooter_nearlunch_KP = 1.8;
    public static double shooter_nearlunch_KD = 0;


    public static double target_X = 142.0;
    public static double target_Y = 142.0;
    public static int autostartstate =0;

    public static Pose botPose = null;

    public static int shooterLongRangeSpeed = 5400, shooterNearRangeSpeed = 3400/*4500*/;
    //自動角度 0~1
    public static double angleControlLong = 0.45 , angleControlNear = 0/*0.52*/;


}
