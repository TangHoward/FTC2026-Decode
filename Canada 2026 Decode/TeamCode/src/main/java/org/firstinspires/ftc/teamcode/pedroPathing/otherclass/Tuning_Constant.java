package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tuning_Constant {
    public static double Shooter_P = 25, Shooter_I = 1, Shooter_D = 0, Shooter_F = 11.7;
    public static double testing_Shooter_Target_RPM = 2550,testing_Forward_Intake_Power =1, testing_Rear_Intake_Power = 0.4;
    public static double servo = 0,angleServo = 0.05;
    public static double lunch_EFFICIENCY = 0.41;
    public static double PREFERRED_ALPHA_DEG = 53; //40~72
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
