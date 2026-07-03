package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tuning_Constant {
    public static double Shooter_P = 25, Shooter_I = 1, Shooter_D = 0, Shooter_F = 11.9;
    public static double testing_Shooter_Target_RPM = 2550,testing_Forward_Intake_Power =1, testing_Rear_Intake_Power = 0.4;
    public static double servo = 0,angleServo = 0.05;
    public static double lunch_EFFICIENCY = 0.41; // 射球效率 0~1 數字越高射越小力 反之 你們知道的
    public static double PREFERRED_ALPHA_DEG = 55; //40~72 //這個理論上調不到 你們幫我測如果機器在近的地方的時候 driver hub上面有字會寫說 計算角度 是幾度
                                                    // 阿如果遠的跟近的都是一樣的那你們就改這裡 然後跟我講 不然去Control_Mode改
                                                    // 阿這個數值的意思是砲台的射球角度 數字越高射越直 數字越小拋越高
    public static double Turret_Tx_P = 0.4;
    public static double Turret_IMU_P = 0.6;
    public static double Turret_IMU_I = 0.0;
    public static double Turret_IMU_D = 0.01;
    public static double Turret_IMU_I_MAX = 5.0;
    public static double Turret_Tx_I = 0.0;
    public static double Turret_Tx_D = 0.0;
    public static double Turret_Tx_I_MAX = 10.0;
    public static double Turret_Tx_MAX_CORR = 15.0;
}
