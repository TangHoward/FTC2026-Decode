    package org.firstinspires.ftc.teamcode.pedroPathing._5TurnServo;

    public class _5TurnServoRegulate {
        private double[] realAngle ={},theoreticalAngle = {};
        public _5TurnServoRegulate(double[] realAngle, double[] theoreticalAngle){
            this.realAngle = realAngle;
            this.theoreticalAngle = theoreticalAngle;
        }
        public double regulate(double position){
            if(realAngle.length > 0|| theoreticalAngle.length > 0) {
                double targetAngle = position * 1800;
                double regulateAngle = interpolate(targetAngle);
                return regulateAngle / 1800;
            }else{
                return position;
            }
        }
        private double interpolate(double targetAngle) {
            int index = java.util.Arrays.binarySearch(realAngle, targetAngle);

            if (index >= 0) return theoreticalAngle[index];

            int insertPoint = -(index + 1);

            if (insertPoint == 0) return theoreticalAngle[0];
            if (insertPoint >= realAngle.length) return theoreticalAngle[realAngle.length - 1];

            double x0 = realAngle[insertPoint - 1];
            double x1 = realAngle[insertPoint];
            double y0 = theoreticalAngle[insertPoint - 1];
            double y1 = theoreticalAngle[insertPoint];

            return y0 + (y1 - y0) * (targetAngle - x0) / (x1 - x0);

        }
    }
