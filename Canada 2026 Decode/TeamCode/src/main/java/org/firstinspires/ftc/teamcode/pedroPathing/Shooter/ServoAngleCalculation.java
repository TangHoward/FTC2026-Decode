package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

public class ServoAngleCalculation {
    public final int bigGear = 194,smallGear = 20;
    public final double min_degree = 40,max_degree = 72;
    public ServoAngleCalculation(){

    };
    public double DegreeToPos(double degree){
                degree -= min_degree;
                degree = degree*bigGear/smallGear;
                degree /= 360;
                return Math.max(Math.min(degree,1),0);
    }
}
