package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

@Configurable
public class All_Calculation {
    //機構最小角度
    public static double startdegree = 40;
    //控制角度的機構的半徑,控制角度齒輪的半徑
    public static double angleControlRadius = 140, angleControlGearRadius =19.94;


    public double calculateServoPosition(double targetRad, double controlServoMaxdegree,int whichzone){

        double startRad = Math.toRadians(startdegree + (whichzone==0? InGameTuning.nearLunchAngleError: InGameTuning.longLunchAngleError));
        double mechRad = targetRad - startRad;
        double servoMaxRad = Math.toRadians(controlServoMaxdegree);

        if (mechRad < 0) mechRad = 0;

        double arcLength = mechRad * angleControlRadius;
        double gearRad = arcLength / angleControlGearRadius;
        double pos = gearRad / servoMaxRad;

        if (pos < 0) pos = 0;
        if (pos > 1) pos = 1;

        return pos;
    }
    //這裡的單位都是SI
    public static double flyWheelRadius = 0.05;
    public static double efficiencyRate = 0.28;

    public static double shooterHeightM = 0.292;
    double[] targetXYZ;
    double[] obstacleXYZ;
    Follower bot;
    public All_Calculation(double[] targetXYZ,Follower bot,double[] obstacleXYZ){
        this.targetXYZ = targetXYZ;
        this.bot =bot;
        this.obstacleXYZ = obstacleXYZ;
    }
    public double calculateElementRadians(double shooterVelocity){
        if (shooterVelocity < 50) return -1;
        double tx = targetXYZ[0] * 0.0254;
        double ty = targetXYZ[1] * 0.0254;
        double tz = targetXYZ[2] * 0.0254;
        double D;
        double g = 9.8;
        double distance=0,h= tz- shooterHeightM;
        double low_angle,high_angle;
        double flyWheelRPS,flyWheelTangentialVelocity, elementSpeed;

        distance = Math.sqrt(Math.pow(bot.getPose().getX()*0.0254 - tx, 2) + Math.pow(bot.getPose().getY()*0.0254 - ty, 2));
        flyWheelRPS = shooterVelocity/28;
        flyWheelTangentialVelocity = flyWheelRPS *flyWheelRadius * 2*Math.PI;
        elementSpeed = flyWheelTangentialVelocity*efficiencyRate;
        /*
        一、基礎軌跡方程式
            物體在忽略空氣阻力下的軌跡公式為：
            y = x * tan(theta) - (g * x^2) / (2 * v0^2 * cos(theta)^2)

        二、數學轉換
            利用三角函數恆等式 sec(theta)^2 = 1 + tan(theta)^2 將公式改寫：
            y = x * tan(theta) - (g * x^2) / (2 * v0^2) * (1 + tan(theta)^2)

        三、整理為一元二次方程式
            設 T = tan(theta)，整理後得到標準形式 aT^2 + bT + c = 0：
            係數如下：
            a = (g * x^2) / (2 * v0^2)
            b = -x
            c = y + (g * x^2) / (2 * v0^2)

        四、判斷是否能到達 (判別式 Delta)
            利用一元二次方程式判別式 (b^2 - 4ac) 來檢查解的存在性。
            化簡後的判斷數值 (Discriminant, D) 為：
            D = v0^4 - g * (g * x^2 + 2 * v0^2 * y)

        判斷法則：
            1. 若 D < 0：
            目標超出射程，無法到達 (無解)。
            2. 若 D = 0：
            恰好到達邊緣，只有唯一角度。
        3. 若 D > 0：
            目標在射程內，有兩個角度可到達 (高拋射與低平射)。

        五、最終角度公式 (求解 Theta)
            若 D >= 0，利用公式解：
            tan(theta) = (v0^2 ± sqrt(D)) / (g * x)

        因此角度為：
            theta_1 = arctan( (v0^2 + sqrt(D)) / (g * x) )  -> 高拋角度
            theta_2 = arctan( (v0^2 - sqrt(D)) / (g * x) )  -> 低射角度
         */

        D = Math.pow(elementSpeed,4)-g*(g*Math.pow(distance,2)+2*Math.pow(elementSpeed,2)*h);
        if(D <0){
            return -1;
        }
        if (distance < 1e-6) return -1;
        low_angle = Math.atan((Math.pow(elementSpeed,2)-Math.sqrt(D))/(g*distance));

        high_angle = Math.atan((Math.pow(elementSpeed,2)+Math.sqrt(D))/(g*distance));
        return low_angle >= Math.toRadians(startdegree) ? low_angle :
                high_angle <= Math.toRadians(60) ? high_angle :
                        -1;
        //return low_angle > Math.toRadians(startdegree)&& low_angle < Math.toRadians(60) ? low_angle : -1;
    }
    public double[] solveShooterRPMAndAngle() {
        double g = 9.8; // m/s^2
        double minDeg = startdegree; // 機構最小角
        double maxDeg = 60;          // 機構最大角
        double stepDeg = 0.5;        // 搜尋解析度（度）

        // === 目標位置（inch → meter）===
        double tx = targetXYZ[0] * 0.0254;
        double ty = targetXYZ[1] * 0.0254;
        double tz = targetXYZ[2] * 0.0254;

        // === 機器人位置（inch → meter）===
        double bx = bot.getPose().getX() * 0.0254;
        double by = bot.getPose().getY() * 0.0254;

        // === 自動計算距離與高度差 ===
        double distance = Math.hypot(tx - bx, ty - by);   // 水平距離
        double height = tz - shooterHeightM;              // 高度差

        if (distance < 1e-6) return new double[]{-1, -1};

        double bestRPM = Double.POSITIVE_INFINITY;
        double bestAngleRad = -1;

        // === 掃描角度，找最小 RPM 解 ===
        for (double deg = minDeg; deg <= maxDeg; deg += stepDeg) {
            double theta = Math.toRadians(deg);
            double denom = distance * Math.tan(theta) - height;
            if (denom <= 0) continue;

            double cos = Math.cos(theta);

            double v2 = (g * distance * distance) /
                    (2 * cos * cos * denom);

            if (v2 <= 0) continue;

            double v = Math.sqrt(v2);

            // === 反算 RPM ===
            double wheelLinearSpeed = v / efficiencyRate;
            double rps = wheelLinearSpeed / (2 * Math.PI * flyWheelRadius);
            double rpm = rps * 60;

            double targetDistance = Math.hypot(targetXYZ[0]- bot.getPose().getX(), targetXYZ[1] - bot.getPose().getY());

            double dirLen = Math.hypot(tx - bx, ty - by);
            boolean blocked = rayBlockedByWallWithHeight(
                    bx, by,
                    (tx - bx) / dirLen, (ty-by)/dirLen,
                    theta,
                    v,
                    obstacleXYZ[0]*0.0254, obstacleXYZ[1]*0.0254,
                    obstacleXYZ[2]*0.0254, obstacleXYZ[3]*0.0254,
                    obstacleXYZ[4]*0.0254
            );

            if (blocked) continue;

            if (rpm < bestRPM ) {
                bestRPM = rpm;
                bestAngleRad = theta;
            }
        }

        if (bestAngleRad < 0 || bestRPM == Double.POSITIVE_INFINITY) {
            return new double[]{-1, -1};
        }

        return new double[]{bestRPM, bestAngleRad};

    }
    private Double rayIntersectWallDistance(
            double rx, double ry,
            double dx, double dy,
            double wx1, double wy1,
            double wx2, double wy2
    ) {
        double sdx = wx2 - wx1;
        double sdy = wy2 - wy1;

        double denom = dx * sdy - dy * sdx;
        if (Math.abs(denom) < 1e-6) return null; // 平行

        double t = ((wx1 - rx) * sdy - (wy1 - ry) * sdx) / denom;
        double u = ((wx1 - rx) * dy - (wy1 - ry) * dx) / denom;

        if (t >= 0 && u >= 0 && u <= 1) {
            return Math.sqrt(
                    Math.pow(t * dx, 2) +
                            Math.pow(t * dy, 2)
            );
        }
        return null;
    }
    private boolean rayBlockedByWallWithHeight(
            double rx, double ry,
            double dx, double dy,
            double theta,
            double v,
            double wx1, double wy1,
            double wx2, double wy2,
            double wallHeight
    ) {
        Double hitDist = rayIntersectWallDistance(
                rx, ry, dx, dy,
                wx1, wy1, wx2, wy2
        );


        if (hitDist == null) return false;

        // 拋體在該距離的高度
        double g = 9.8;
        double x = hitDist* Math.cos(theta);
        double z =
                shooterHeightM
                        + x * Math.tan(theta)
                        - (g * x * x) / (2 * v * v * Math.cos(theta) * Math.cos(theta));

        return z <= wallHeight;
    }

}
