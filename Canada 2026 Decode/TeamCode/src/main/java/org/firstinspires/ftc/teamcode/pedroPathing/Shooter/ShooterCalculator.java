package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

import com.pedropathing.follower.Follower;

/**
 * ============================================================
 * ShooterCalculator  –  Team 28008 Sea Master
 * ============================================================
 *
 * 使用方式（在你的 OpMode 裡）：
 *
 *   // 1. 建立 Calculator（一次就好）
 *   ShooterCalculator shooter = new ShooterCalculator(follower);
 *
 *   // 2. 設定目標位置（場地座標，inches）
 *   shooter.setGoal(goalX, goalY, goalHeightInches);
 *
 *   // 3. 每個 loop 呼叫 update()，取得結果
 *   ShooterCalculator.ShootResult r = shooter.update();
 *   if (r.valid) {
 *       double launchAngleRad = r.launchAngle;   // 仰角 (radians)
 *       double flywheelRPM    = r.flywheelRPM;   // 命令飛輪的轉速
 *       double turretOffset   = r.turretOffset;  // 砲塔偏移 (radians)
 *   }
 *
 * ============================================================
 * 角度策略：高拋低射速
 *   在 [PREFERRED_ALPHA_MIN, PREFERRED_ALPHA_MAX] 範圍內，
 *   使用仰角 PREFERRED_ALPHA_MAX（最高仰角 = 最低飛輪速度）。
 *   若目標太近導致最高仰角無法到達，自動往下找可行角度。
 * ============================================================
 */
public class ShooterCalculator {

    // ------------------------------------------------------------------
    // 物理常數
    // ------------------------------------------------------------------
    /** 重力加速度 (inches/s²) */
    public static final double GRAVITY = 386.1;

    // ------------------------------------------------------------------
    // 飛輪參數  ←  如需調整只改這裡
    // ------------------------------------------------------------------
    /** 飛輪半徑 (inches)，96mm 直徑 */
    public static final double FLYWHEEL_RADIUS_IN = 48.0 / 25.4;   // 1.8898 in

    /** 馬達空載最高轉速 (RPM)，GoBILDA 5203-2402-0001 */
    public static final double MOTOR_MAX_RPM = 4500;

    /**
     * 發射效率係數（0~1）。
     * 飛輪表面速度 × efficiency = 球的實際出膛速度。
     * 根據實測調整（壓縮量越大通常越高，建議先用 0.85）。
     */
    public static final double LAUNCH_EFFICIENCY = 0.85;

    // ------------------------------------------------------------------
    // 仰角策略參數  ←  如需調整只改這裡
    // ------------------------------------------------------------------
    /** 高拋目標仰角（最優先嘗試），degrees → radians */
    public static final double PREFERRED_ALPHA_MAX = Math.toRadians(55.0);
    /** 仰角搜尋下限（低於此角放棄高拋，改用最低可行角） */
    public static final double PREFERRED_ALPHA_MIN = Math.toRadians(40.0);
    /** 機構物理最小仰角限制 */
    public static final double MECH_ALPHA_MIN      = Math.toRadians(40.0);
    /** 機構物理最大仰角限制 */
    public static final double MECH_ALPHA_MAX      = Math.toRadians(55.0);

    // ------------------------------------------------------------------
    // 結果容器
    // ------------------------------------------------------------------
    public static class ShootResult {
        /** 最終發射仰角 (radians)，你再轉成 Hood 伺服位置 */
        public double  launchAngle  = 0.0;
        /** 最終發射速度 (inches/s) */
        public double  launchSpeed  = 0.0;
        /** 命令飛輪的轉速 (RPM) */
        public double  flywheelRPM  = 0.0;
        /** 砲塔偏移角 (radians)；靜止時為 0 */
        public double  turretOffset = 0.0;
        /** 計算是否成功 */
        public boolean valid        = false;
        /** 失敗原因（valid=false 時有內容） */
        public String  errorMessage = "";
    }

    // ------------------------------------------------------------------
    // 內部狀態
    // ------------------------------------------------------------------
    private final Follower follower;

    /** 目標場地 X 座標 (inches) */
    private double goalFieldX = 0.0;
    /** 目標場地 Y 座標 (inches) */
    private double goalFieldY = 0.0;
    /** 目標高度，相對於飛輪出口 (inches) */
    private double goalHeightRelative = 0.0;

    /** 仰角 Clamp 下限（速度補償用） */
    private double minAngleForComp = MECH_ALPHA_MIN;
    /** 仰角 Clamp 上限（速度補償用） */
    private double maxAngleForComp = MECH_ALPHA_MAX;

    // ------------------------------------------------------------------
    // 建構子
    // ------------------------------------------------------------------
    /**
     * @param follower  你的 Pedro Pathing Follower 實例
     */
    public ShooterCalculator(Follower follower) {
        this.follower = follower;
    }

    // ------------------------------------------------------------------
    // 設定目標（在射擊前或目標改變時呼叫）
    // ------------------------------------------------------------------
    /**
     * 設定目標場地座標與高度。
     *
     * @param fieldX          目標場地 X (inches)
     * @param fieldY          目標場地 Y (inches)
     * @param heightRelative  目標相對於飛輪出口的高度 (inches)，向上為正
     */
    public void setGoal(double fieldX, double fieldY, double heightRelative) {
        this.goalFieldX          = fieldX;
        this.goalFieldY          = fieldY;
        this.goalHeightRelative  = heightRelative;
    }

    /**
     * （可選）覆寫速度補償的仰角 Clamp 範圍。
     * 若不呼叫，預設使用 MECH_ALPHA_MIN / MECH_ALPHA_MAX。
     */
    public void setAngleClamp(double minRad, double maxRad) {
        this.minAngleForComp = minRad;
        this.maxAngleForComp = maxRad;
    }

    // ------------------------------------------------------------------
    // 主要更新方法（每個 loop 呼叫）
    // ------------------------------------------------------------------
    /**
     * 讀取 Follower 的當前位置與速度，計算並回傳射擊參數。
     * 若尚未呼叫 setGoal()，回傳 valid=false。
     *
     * @return ShootResult  含仰角、速度、RPM、砲塔偏移
     */
    public ShootResult update() {
        ShootResult result = new ShootResult();

        // --- 從 Follower 取得機器人當前位置 ---
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        // --- 計算水平距離與目標方向角 ---
        double dx = goalFieldX - robotX;
        double dy = goalFieldY - robotY;
        double horizontalDist = Math.sqrt(dx * dx + dy * dy);
        double thetaLine      = Math.atan2(dy, dx);  // 機器人到目標的方向角

        if (horizontalDist < 1.0) {
            result.errorMessage = "距離目標太近 (" + String.format("%.1f", horizontalDist) + " in)";
            return result;
        }

        // --- A. 靜止基礎計算（高拋角策略） ---
        ShootResult baseResult = calculateHighArc(horizontalDist, goalHeightRelative);
        if (!baseResult.valid) {
            result.errorMessage = "無法計算高拋彈道: " + baseResult.errorMessage;
            return result;
        }

        // --- B. 速度補償 ---
        // 從 Follower 取得機器人速度（Pedro Pathing 提供）
        double robotVelX   = follower.getVelocity().getXComponent();
        double robotVelY   = follower.getVelocity().getYComponent();
        double velMag      = Math.sqrt(robotVelX * robotVelX + robotVelY * robotVelY);

        // 若速度夠小，略過補償直接用靜止結果
        if (velMag < 0.5) {
            baseResult.turretOffset = 0.0;
            return baseResult;
        }

        double robotVelTheta = Math.atan2(robotVelY, robotVelX);

        return applyVelocityCompensation(
                baseResult,
                horizontalDist,
                goalHeightRelative,
                velMag,
                robotVelTheta,
                thetaLine
        );
    }

    // ------------------------------------------------------------------
    // A. 高拋角計算
    // ------------------------------------------------------------------

    /**
     * 高拋角策略：
     * 從 PREFERRED_ALPHA_MAX 開始往下掃，找到第一個物理可行的仰角。
     * 這樣可確保在機器人能力範圍內盡量使用高仰角（低飛輪速度）。
     *
     * @param x  水平距離 (inches)
     * @param y  目標相對高度 (inches)
     * @return   ShootResult，含最高可行仰角和對應的 v0、RPM
     */
    public ShootResult calculateHighArc(double x, double y) {
        // 從最高仰角往下掃，步進 0.5°
        double stepRad = Math.toRadians(0.5);

        for (double alpha = PREFERRED_ALPHA_MAX; alpha >= MECH_ALPHA_MIN; alpha -= stepRad) {
            ShootResult r = solveForAlpha(x, y, alpha);
            if (r.valid) {
                // 若 RPM 超出馬達上限，仰角太高也不行，繼續往下
                if (r.flywheelRPM > MOTOR_MAX_RPM) continue;
                return r;
            }
        }

        ShootResult fail = new ShootResult();
        fail.errorMessage = "所有仰角均無法到達目標 (x=" + String.format("%.1f",x)
                + " y=" + String.format("%.1f",y) + ")";
        return fail;
    }

    // ------------------------------------------------------------------
    // B. 速度補償
    // ------------------------------------------------------------------

    private ShootResult applyVelocityCompensation(
            ShootResult base,
            double x,
            double y,
            double robotVelMag,
            double robotVelTheta,
            double thetaLine) {

        ShootResult result = new ShootResult();

        double v0    = base.launchSpeed;
        double alpha = base.launchAngle;

        // B1：分解為徑向 / 切向
        double thetaDiff = robotVelTheta - thetaLine;
        double Vrr = -Math.cos(thetaDiff) * robotVelMag;  // 徑向（+ = 靠近目標）
        double Vrt =  Math.sin(thetaDiff) * robotVelMag;  // 切向

        // B2：飛行時間
        double flightTime = x / (v0 * Math.cos(alpha));
        if (flightTime <= 0) {
            result.errorMessage = "飛行時間異常";
            return result;
        }

        // B3：補償後水平速度
        double VxBase        = x / flightTime;
        double VxCompensated = VxBase + Vrr;
        double VxNew         = Math.sqrt(VxCompensated * VxCompensated + Vrt * Vrt);

        // B4：保留垂直速度
        double Vy = v0 * Math.sin(alpha);

        // B5：新仰角（含 Clamp）
        double alphaNew = Math.atan2(Vy, VxNew);
        alphaNew = clamp(alphaNew, minAngleForComp, maxAngleForComp);

        // B6：新距離 & 新速度
        double xNew = VxNew * flightTime;
        ShootResult newSpeed = solveForAlpha(xNew, y, alphaNew);
        if (!newSpeed.valid) {
            result.errorMessage = "補償後速度計算失敗: " + newSpeed.errorMessage;
            return result;
        }

        // B7：砲塔偏移
        double turretOffset = 0.0;
        if (Math.abs(VxCompensated) > 1e-9) {
            turretOffset = Math.atan2(Vrt, VxCompensated);
        }

        result.launchAngle  = alphaNew;
        result.launchSpeed  = newSpeed.launchSpeed;
        result.flywheelRPM  = newSpeed.flywheelRPM;
        result.turretOffset = turretOffset;
        result.valid        = true;
        return result;
    }

    // ------------------------------------------------------------------
    // C. 已知 alpha，計算 v0 和 RPM
    // ------------------------------------------------------------------

    /**
     * 給定仰角 alpha，計算達到 (x, y) 所需的 v0 與 RPM。
     *
     * @param x      水平距離 (inches)
     * @param y      相對高度 (inches)
     * @param alpha  仰角 (radians)
     * @return       ShootResult（launchSpeed、flywheelRPM 有效）
     */
    public ShootResult solveForAlpha(double x, double y, double alpha) {
        ShootResult result = new ShootResult();

        double cosA  = Math.cos(alpha);
        double tanA  = Math.tan(alpha);
        double denom = 2.0 * cosA * cosA * (x * tanA - y);

        if (denom <= 0) {
            result.errorMessage = "alpha=" + formatDeg(alpha)
                        + " 在此目標物理上不可行 (denom=" + String.format("%.3f", denom) + ")";
            return result;
        }

        double v0  = Math.sqrt(GRAVITY * x * x / denom);
        double rpm = launchSpeedToRPM(v0);

        result.launchAngle  = alpha;
        result.launchSpeed  = v0;
        result.flywheelRPM  = rpm;
        result.turretOffset = 0.0;
        result.valid        = true;
        return result;
    }

    // ------------------------------------------------------------------
    // RPM 換算
    // ------------------------------------------------------------------

    /**
     * 球的出膛速度 → 飛輪 RPM。
     *
     * 公式：RPM = v0 / (2π * r * efficiency) * 60
     *
     * @param launchSpeedInPerSec  出膛速度 (inches/s)
     * @return 飛輪轉速 (RPM)
     */
    public static double launchSpeedToRPM(double launchSpeedInPerSec) {
        // v_surface = v_ball / efficiency
        // RPM = v_surface / (2π*r) * 60
        return (launchSpeedInPerSec / LAUNCH_EFFICIENCY)
                / (2.0 * Math.PI * FLYWHEEL_RADIUS_IN)
                * 60.0;
    }

    /**
     * 飛輪 RPM → 球的出膛速度 (inches/s)。
     * （方便從外部反查，例如在 Telemetry 顯示）
     */
    public static double rpmToLaunchSpeed(double rpm) {
        return rpm * (2.0 * Math.PI * FLYWHEEL_RADIUS_IN) / 60.0 * LAUNCH_EFFICIENCY;
    }

    // ------------------------------------------------------------------
    // 工具方法
    // ------------------------------------------------------------------

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public static String formatDeg(double radians) {
        return String.format("%.2f°", Math.toDegrees(radians));
    }
}