package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class TeleOpHeadingPD_Cam {
    // 宣告 鏡頭修正PD 數值
    private double kP, kD;
    // 目標april tag 數字
    private int targetNumber;
    // 需要PD計算的數值
    private double previousError = 0.0;
    private double previousTime = 0.0;
    private Hardware hardware ;
    private List<AprilTagDetection> detectionsTags  = new ArrayList<>();

    private boolean targetFound = false;
    //設定數值的副程式
    public TeleOpHeadingPD_Cam(double kP, double kD, int targetNumber, Hardware hardware){
        this.kP = kP;
        this.kD = kD;
        this.targetNumber = targetNumber;
        this.hardware = hardware;
    }
    public void setTargetNumber(int targetNumber){
        this.targetNumber = targetNumber;
    }
    // 設定PD 的副程式
    public void setCoefficients(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
    }
    // 利用誤差和PD數值 去計算所需的力量
    public double calculateTurnPower(double currentTime){
        update();
        double error = 0,derivative;
        double deltaTime = currentTime - previousTime;
        targetFound = false;
        //判斷是否有目標數字
        for (AprilTagDetection detection : detectionsTags){
            if(detection.id == (int)targetNumber){
                error = 640 - detection.center.x;
                targetFound = true;
                break; // 找到目標了，直接跳出迴圈，不要管其他的 Tag
            }
        }

        if (!targetFound) {
            return 0.0;
        }
        derivative = deltaTime > 0 ? (error - previousError) / deltaTime : 0;
        double pTerm = kP * error;
        double dTerm = kD * derivative;

        previousError = error;
        previousTime = currentTime;

        return Math.min(Math.max(pTerm + dTerm,-0.7),0.7);
    }
    // 更新april tag
    private void update() {
        detectionsTags = hardware.aprilTag.getDetections();
    }
    // 確認使否有找到目標april tag
    public boolean foundTarget() {
        update();
        if(targetFound){
            return targetFound;
        }else {
            for (AprilTagDetection detection : detectionsTags) {
                if (detection.id == (int) targetNumber) {
                    return  true;
                }
            }
        }
        return false;
    }
}
