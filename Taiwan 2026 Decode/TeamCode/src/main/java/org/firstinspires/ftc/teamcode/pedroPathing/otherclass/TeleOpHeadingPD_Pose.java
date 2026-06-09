package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

public class TeleOpHeadingPD_Pose {
    //部分程式一樣 請見TeleOpHeadingPD_Cam
    private double Kp = 0, Kd = 0;
    private double targetHeadingRadians = 0;

    private double previousError = 0.0;
    private double previousTime = 0.0;

    public TeleOpHeadingPD_Pose(double kP, double kD, double targetHeading) {
        this.Kp = kP;
        this.Kd = kD;
        this.targetHeadingRadians = targetHeading;
    }

    public void setTargetHeading(double targetHeading){
        this.targetHeadingRadians = targetHeading;
    }
    public void setCoefficients(double kP, double kD) {
        this.Kp = kP;
        this.Kd = kD;
    }
    public double calculateTurnPower(double currentHeading, double currentTime){

        double error =   angleWrap(targetHeadingRadians - currentHeading);
        double deltaTime = currentTime - previousTime;

        double derivative =0;

        double returnvalue = 0;
        derivative = deltaTime > 0 ? (error-previousError) / deltaTime : 0;

         double pTerm = Kp * error;
         double dTerm = Kd * derivative;

         previousError = error;
         previousTime = currentTime;

         returnvalue = Math.min(Math.max(pTerm + dTerm, -0.7),0.7);
         return returnvalue;
    }
    //去解決機器轉一整圈的問題
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
