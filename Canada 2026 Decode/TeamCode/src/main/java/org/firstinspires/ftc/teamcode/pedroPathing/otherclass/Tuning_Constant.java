package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tuning_Constant {
    public static double Shooter_P_Far = 120, Shooter_I_Far = 0, Shooter_D_Far = 0, Shooter_F_Far = 11.4;
    public static double Shooter_P_Close = 100, Shooter_I_Close = 0, Shooter_D_Close = 0, Shooter_F_Close = 11.9;

    public static double testing_Shooter_Target_RPM = 2550,testing_Forward_Intake_Power =1, testing_Rear_Intake_Power = 0.4;
    public static double servo = 0,angleServo = 0.05;
    public static double lunch_EFFICIENCY = 0.42;
    public static double PREFERRED_ALPHA_DEG = 53; //40~72
    public static double Turret_Tx_P = 0;
    public static double Turret_IMU_P = 0.6;
    public static double Turret_IMU_I = 0.0;
    public static double Turret_IMU_D = 0.01;
    public static double Turret_IMU_I_MAX = 5.0;
    public static double Turret_Tx_I = 1;
    public static double Turret_Tx_D = 0.0005;
    public static double Turret_Tx_I_MAX = 10;
    // 進入「精修 I」的誤差門檻(度),需小於 Turret_Tx_Deadband_Exit_Deg
    public static double Turret_Tx_I_Fine_Deg = 4;

    // 精修用的第二組 I 增益,通常比 Turret_Tx_I 大,讓誤差在小範圍時能更快收斂到 0
    public static double Turret_Tx_I_Fine = 0.5;
    public static double Turret_Tx_Deadband_Deg = 0.5;
    public static double Turret_Tx_Deadband_Exit_Deg = Turret_Tx_Deadband_Deg * 1.75;
    public static double Turret_Tx_MAX_CORR = 15.0;
    public static double Turret_Tx_I_Freeze_Speed_InPerSec = 1.5;

    // ── Limelight 視覺定位：動態品質過濾 ─────────────────────
    /** 平均 tag 距離超過這個值（inches）就直接拒絕本次視覺讀值 */
    public static double Vision_Max_Trust_Dist_In = 156.0;
    /** 砲台角速度超過這個值（deg/s）就拒絕本次視覺讀值（動態模糊風險） */
    public static double Vision_Max_Turret_AngVel_DegS = 360.0;
    /** 底盤角速度超過這個值（deg/s）就拒絕本次視覺讀值（動態模糊風險） */
    public static double Vision_Max_Chassis_AngVel_DegS = 360.0;
    /** 底盤平移速度超過這個值（inches/s）就拒絕本次視覺讀值（移動模糊 + 視覺延遲風險） */
    public static double Vision_Max_Chassis_LinearVel_InS = 3.0;
    /** 時序一致性檢查：連續兩幀候選位置的容許差距（inches） */
    public static double Vision_Consistency_Tolerance_In = 6.0;
    /** 時序一致性檢查：兩幀之間的最大時間間隔（ms），超過就視為不連續 */
    public static double Vision_Consistency_Window_Ms = 300.0;

    // ── Limelight 視覺定位：漸進式融合（避免瞬間平移）───────────
    /** 單次融合允許的最大位移量（inches），超過就分成多個 loop 逐漸收斂 */
    public static double Vision_Max_Correction_Per_Fuse_In = 6.0;
    /** 單次融合允許的最大角度修正量（degrees） */
    public static double Vision_Max_Heading_Correction_Deg = 8.0;
    /** 融合時視覺權重的上限（0~1），避免單一低品質讀值就大幅覆蓋 Pedro 估計 */
    public static double Vision_Max_Fuse_Weight = 0.6;
    public static double number1 = 72,number2 = 0.42;
    public static double x_localizeTest_startPose = 72,y_localizeTest_startPose = 72, r_localizeTest_startPose = 90;
}