package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Configurable_Constants {
    // 駕駛參數
    public static double slow_mode_mutiplier = 0.5;

    // PD 控制器參數 (即時調校)
    public static double heading_kp = 1;
    public static double heading_kd = 0.05;

    public static boolean autoshoot = false;

    public static double shooter_longlunch_F = 14.9;
    public static double shooter_longlunch_KP = 2.8;

    public static double shooter_nearlunch_F = 14.2;
    public static double shooter_nearlunch_KP = 1.8;



    public static double target_X = 144.0;
    public static double target_Y = 144.0;
    public static int autostartstate =0;

    public static Pose botPose = null;

    public static int shooterLongRangeSpeed = 4500, shooterNearRangeSpeed = 3800;
    public static double angleControlLong = 0.56 , angleControlNear = 0.37;


}
