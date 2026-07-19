package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

/**
 * ============================================================
 * ShooterCalculator – 查表版（Team 28008 Sea Master）
 * ============================================================
 * isFar = false：依「水平距離」線性插值查表，取得伺服角度與 RPM。
 * isFar = true ：固定角度 FAR_ANGLE_DEG / 固定 RPM FAR_RPM，不查表。
 *
 * ⚠️ result.servoAngleDeg 可以直接丟進
 *     servoAngleCalculation.DegreeToPos(result.servoAngleDeg)
 *    不需要再做 90 - angle 的轉換。
 * ============================================================
 */
public class ShooterCalculator {

    // ------------------------------------------------------------------
    // 查表資料（依距離由小到大排序）：{ distance, servoAngleDeg, RPM }
    // ------------------------------------------------------------------
    private static final double[][] LOOKUP_TABLE = {
            {65.38172528, 45, 2500},
            {76.12726187, 46, 2500},
            {86.35930755, 45.5, 2500},
            {93.43446901, 42, 2500},
            {99.72462083, 40, 2500},
//            {80.692069,   42, 2500},
            /*{87.11739206, 44, 2500},
            {92.4896751,  44, 2650},
            {98.6927049,  43, 2700},
            {104.2874873, 45, 2750},
            {106.2905922, 47, 2800},
            {110.8801154, 47, 2900},*/
    };

    /** isFar = true 時的固定伺服角度 (degrees) */
    public static final double FAR_ANGLE_DEG = 55.0;
    /** isFar = true 時的固定 RPM */
    public static final double FAR_RPM = 3400;

    // ------------------------------------------------------------------
    // 飛輪參數（保留給 rpmToLaunchSpeed，給 Turret 速度補償用）
    // ------------------------------------------------------------------
    public static final double FLYWHEEL_RADIUS_IN = 48.0 / 25.4; // 96mm 直徑

    // ------------------------------------------------------------------
    // 結果容器
    // ------------------------------------------------------------------
    public static class ShootResult {
        /** 伺服角度 (degrees)，可直接送進 DegreeToPos()，不需再轉換 */
        public double  servoAngleDeg = 0.0;
        /** 命令飛輪的轉速 (RPM) */
        public double  flywheelRPM   = 0.0;
        /** 換算出的出膛速度 (inches/s)，供 Turret 速度補償使用 */
        public double  launchSpeed   = 0.0;
        /** 數學慣例仰角 (radians, 0°=水平)，供 Turret 速度補償使用 */
        public double  launchAngleRad = 0.0;
        /** 計算是否成功 */
        public boolean valid = false;
        /** 失敗原因 */
        public String  errorMessage = "";
    }

    private final Follower follower;

    private double goalFieldX = 0.0;
    private double goalFieldY = 0.0;

    /** true = 遠程固定值模式，false = 查表模式。由 setFarMode() 設定 */
    private boolean isFar = false;

    public ShooterCalculator(Follower follower) {
        this.follower = follower;
    }

    /**
     * 設定目標場地座標。height 參數保留參數相容性，查表模式不使用高度。
     */
    public void setGoal(double fieldX, double fieldY, double heightRelative) {
        this.goalFieldX = fieldX;
        this.goalFieldY = fieldY;
    }

    /** 設定目前是否為遠程模式（true = 固定值，false = 查表） */
    public void setFarMode(boolean isFar) {
        this.isFar = isFar;
    }

    public boolean isFarMode() {
        return isFar;
    }

    public ShootResult update() {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double dx = goalFieldX - robotX;
        double dy = goalFieldY - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        ShootResult result = isFar
                ? fixedFarResult()
                : lookupByDistance(distance);

        if (result.valid) {
            result.launchSpeed = rpmToLaunchSpeed(result.flywheelRPM);
            // 伺服慣例(0=垂直,90=水平) → 數學慣例(0=水平,90=垂直)
            result.launchAngleRad = Math.toRadians(90.0 - result.servoAngleDeg);
        }
        return result;
    }

    private ShootResult fixedFarResult() {
        ShootResult result = new ShootResult();
        result.servoAngleDeg = FAR_ANGLE_DEG;
        result.flywheelRPM   = FAR_RPM;
        result.valid = true;
        return result;
    }

    /**
     * 依水平距離線性插值查表。距離超出表格範圍時，直接夾在邊界值（不外插）。
     */
    private ShootResult lookupByDistance(double distance) {
        ShootResult result = new ShootResult();
        int n = LOOKUP_TABLE.length;

        if (distance <= LOOKUP_TABLE[0][0]) {
            result.servoAngleDeg = LOOKUP_TABLE[0][1];
            result.flywheelRPM   = LOOKUP_TABLE[0][2];
            result.valid = true;
            return result;
        }
        if (distance >= LOOKUP_TABLE[n - 1][0]) {
            result.servoAngleDeg = LOOKUP_TABLE[n - 1][1];
            result.flywheelRPM   = LOOKUP_TABLE[n - 1][2];
            result.valid = true;
            return result;
        }

        for (int i = 0; i < n - 1; i++) {
            double d0 = LOOKUP_TABLE[i][0];
            double d1 = LOOKUP_TABLE[i + 1][0];
            if (distance >= d0 && distance <= d1) {
                double t = (distance - d0) / (d1 - d0);
                result.servoAngleDeg = LOOKUP_TABLE[i][1] + (LOOKUP_TABLE[i + 1][1] - LOOKUP_TABLE[i][1]) * t;
                result.flywheelRPM   = LOOKUP_TABLE[i][2] + (LOOKUP_TABLE[i + 1][2] - LOOKUP_TABLE[i][2]) * t;
                result.valid = true;
                return result;
            }
        }

        result.errorMessage = "查表失敗，距離=" + distance;
        return result;
    }

    /**
     * RPM → 出膛速度 (inches/s)，僅供 Turret 速度補償使用。
     */
    public static double rpmToLaunchSpeed(double rpm) {
        return rpm * (2.0 * Math.PI * FLYWHEEL_RADIUS_IN) / 60.0 * Tuning_Constant.lunch_EFFICIENCY;
    }
}