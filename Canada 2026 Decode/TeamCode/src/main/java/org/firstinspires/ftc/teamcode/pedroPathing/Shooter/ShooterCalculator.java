package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

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
 *       double launchAngleRad = r.launchAngle;   // 仰角 (radians)，數學標準慣例
 *       double flywheelRPM    = r.flywheelRPM;   // 命令飛輪的轉速
 *   }
 *
 * ============================================================
 * ⚠️ 角度慣例說明（重要）─────────────────────────────────────
 *   本檔案內部全部使用「數學標準慣例」：0°=水平射出，90°=垂直向上。
 *   這是拋物線公式（solveForAlpha 等）的計算基礎，不可更改。
 *
 *   你的伺服機構慣例是相反的：0°=垂直向上，90°=水平向前。
 *   因此：
 *     - MECH_ALPHA_MIN/MAX 已經換算成數學慣例（用 90°-你的角度 算出來），
 *       不要直接套用機構角度數字。
 *     - 從 update() 拿到 r.launchAngle 後，要送進伺服前，
 *       必須在 Control_Mode 做轉換：
 *         servoAngleDeg = 90.0 - Math.toDegrees(r.launchAngle);
 *         hardware.angleController.setPosition(
 *             servoAngleCalculation.DegreeToPos(servoAngleDeg));
 *       ShooterCalculator 本身不做這個轉換。
 * ============================================================
 * 角度策略：偏好角度優先，雙向擴散搜索
 *   在 [MECH_ALPHA_MIN, MECH_ALPHA_MAX] 範圍內（數學慣例），
 *   從 Tuning_Constant.PREFERRED_ALPHA_DEG 開始，
 *   同時往更高角度和更低角度交替擴散掃描，
 *   找到最接近偏好角度的可行解。
 *   MECH_ALPHA_MIN / MECH_ALPHA_MAX 為硬邊界，絕對不超過。
 *   ⚠️ Tuning_Constant.PREFERRED_ALPHA_DEG 也必須填數學慣例角度
 *      （若你習慣用機構角度思考，記得先換算：90°-機構角度）
 * ============================================================
 * 障礙物檢查：直線形障礙牆
 *   牆為一條無限長直線（由 WALL_POINT_1、WALL_POINT_2 定義），
 *   高度範圍 [0, WALL_HEIGHT_IN]（不含球半徑）。
 *   掃描仰角時，若彈道在穿越牆所在直線的水平位置，
 *   球心高度低於安全高度（牆高 - 球半徑），則此仰角不可用，
 *   繼續往外擴散找下一個角度。
 * ============================================================
 * 速度補償開關
 *   預設「關閉」。機器人移動時是否要根據速度修正發射角度/RPM，
 *   由 setVelocityCompensationEnabled(true/false) 控制。
 *   關閉時，update() 一律回傳偏好角度搜索的靜止解。
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
    public static final double MOTOR_MAX_RPM = 3500;

    /**
     * 發射效率係數（0~1）的存取方法。
     * 每次需要時都直接讀 Tuning_Constant.lunch_EFFICIENCY，
     * 確保 Dashboard 調整能即時生效（不要複製成自己的 static 變數）。
     */
    private static double getLaunchEfficiency() {
        return Tuning_Constant.lunch_EFFICIENCY;
    }

    // ------------------------------------------------------------------
    // 仰角策略參數  ←  如需調整只改這裡（皆為數學標準慣例：0°=水平，90°=垂直）
    // ------------------------------------------------------------------
    /**
     * 機構物理最小仰角限制（硬邊界，絕對不能超過）。
     * 數學慣例值 = 90° - 你的伺服機構角度上限。
     * 你的機構：72°（伺服慣例，較平/較前）→ 數學慣例 = 90-72 = 18°
     */
    public static final double MECH_ALPHA_MIN = Math.toRadians(90.0 - 72.0); // = 18°
    /**
     * 機構物理最大仰角限制（硬邊界，絕對不能超過）。
     * 數學慣例值 = 90° - 你的伺服機構角度下限。
     * 你的機構：40°（伺服慣例，較陡/較高）→ 數學慣例 = 90-40 = 50°
     */
    public static final double MECH_ALPHA_MAX = Math.toRadians(90.0 - 40.0); // = 50°

    // ------------------------------------------------------------------
    // 障礙物（牆）參數  ←  如需調整只改這裡
    // ------------------------------------------------------------------
    /** 牆所在直線上的第一個點（Pedro 場地座標，inches） */
    private static final double WALL_POINT_1_X = 119.33900364520048;
    private static final double WALL_POINT_1_Y = 143.25637910085055;
    /** 牆所在直線上的第二個點（Pedro 場地座標，inches） */
    private static final double WALL_POINT_2_X = 139.63547995139731;
    private static final double WALL_POINT_2_Y = 115.08140947752125;

    /** 牆的物理高度 (inches)，不含球半徑 */
    public static final double WALL_HEIGHT_IN = 38.75;

    /** 球的直徑 (inches)，13cm */
    public static final double BALL_DIAMETER_IN = 13.0 / 2.54;
    /** 球的半徑 (inches) */
    public static final double BALL_RADIUS_IN   = BALL_DIAMETER_IN / 2.0;

    /**
     * 球心通過牆位置時的安全高度上限 (inches)。
     * 球心必須高於這個值，球的頂部才不會碰到牆頂。
     */
    public static final double WALL_SAFE_HEIGHT_IN = WALL_HEIGHT_IN - BALL_RADIUS_IN;

    // 牆的直線方程式係數：A*x + B*y + C = 0（建構子中算好，避免重複計算）
    private static final double WALL_A = WALL_POINT_2_Y - WALL_POINT_1_Y;
    private static final double WALL_B = WALL_POINT_1_X - WALL_POINT_2_X;
    private static final double WALL_C = -(WALL_A * WALL_POINT_1_X + WALL_B * WALL_POINT_1_Y);

    // ------------------------------------------------------------------
    // 結果容器
    // ------------------------------------------------------------------
    public static class ShootResult {
        /** 最終發射仰角 (radians)，數學標準慣例：0°=水平，90°=垂直。
         *  送進伺服前記得換算：servoAngleDeg = 90 - toDegrees(launchAngle) */
        public double  launchAngle  = 0.0;
        /** 最終發射速度 (inches/s) */
        public double  launchSpeed  = 0.0;
        /** 命令飛輪的轉速 (RPM) */
        public double  flywheelRPM  = 0.0;
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

    /** 上一次計算成功的結果（計算失敗時回傳此值） */
    private ShootResult lastValidResult = null;

    /** 仰角 Clamp 下限（速度補償用，數學慣例） */
    private double minAngleForComp = MECH_ALPHA_MIN;
    /** 仰角 Clamp 上限（速度補償用，數學慣例） */
    private double maxAngleForComp = MECH_ALPHA_MAX;

    /**
     * 由 setPreferredAngle() 設定的偏好角度（radians，數學慣例）。
     * null 表示尚未透過程式設定，此時改讀 Tuning_Constant.PREFERRED_ALPHA_DEG。
     */
    private Double preferredAngleRad = null;

    /**
     * 速度補償開關。預設關閉（false）。
     * 開啟後，update() 會在機器人有速度時自動修正發射角度與 RPM。
     */
    private boolean velocityCompensationEnabled = false;

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
     * （可選）覆寫速度補償的仰角 Clamp 範圍（數學慣例：0°=水平，90°=垂直）。
     * 若不呼叫，預設使用 MECH_ALPHA_MIN / MECH_ALPHA_MAX。
     */
    public void setAngleClamp(double minRad, double maxRad) {
        this.minAngleForComp = minRad;
        this.maxAngleForComp = maxRad;
    }

    /**
     * 設定偏好角度（度數，數學慣例：0°=水平，90°=垂直）。
     * 若你習慣用伺服機構角度思考（0°=垂直，90°=水平），
     * 呼叫前先換算：setPreferredAngle(90.0 - 機構角度)
     *
     * 呼叫後，calculatePreferredArc 會以此角度為中心往兩側擴散，
     * 優先級高於 Tuning_Constant.PREFERRED_ALPHA_DEG。
     * 超出 [MECH_ALPHA_MIN, MECH_ALPHA_MAX] 的值會自動 clamp。
     *
     * 若要恢復讀 Tuning_Constant，呼叫 resetPreferredAngle()。
     *
     * @param degrees  偏好仰角（度數，數學慣例）
     */
    public void setPreferredAngle(double degrees) {
        this.preferredAngleRad = clamp(Math.toRadians(degrees), MECH_ALPHA_MIN, MECH_ALPHA_MAX);
    }

    /**
     * 清除由 setPreferredAngle() 設定的值，恢復讀取 Tuning_Constant.PREFERRED_ALPHA_DEG。
     */
    public void resetPreferredAngle() {
        this.preferredAngleRad = null;
    }

    /**
     * 取得當前生效的偏好角度（度數，數學慣例：0°=水平，90°=垂直）。
     * 方便在 Telemetry 顯示目前用的是哪個來源。
     */
    public double getPreferredAngleDeg() {
        if (preferredAngleRad != null) {
            return Math.toDegrees(preferredAngleRad);
        }
        return Tuning_Constant.PREFERRED_ALPHA_DEG;
    }

    /**
     * 開啟/關閉速度補償。預設關閉。
     * 開啟後，機器人移動時 update() 會根據當前速度修正發射角度和 RPM，
     * 讓球在飛行中抵銷機器人移動造成的偏移。
     * 關閉時，一律回傳偏好角度搜索得到的靜止解（不論機器人是否在移動）。
     *
     * @param enabled true=開啟速度補償，false=關閉
     */
    public void setVelocityCompensationEnabled(boolean enabled) {
        this.velocityCompensationEnabled = enabled;
    }

    /**
     * 查詢速度補償目前是否開啟。方便在 Telemetry 顯示狀態。
     */
    public boolean isVelocityCompensationEnabled() {
        return velocityCompensationEnabled;
    }

    // ------------------------------------------------------------------
    // 主要更新方法（每個 loop 呼叫）
    // ------------------------------------------------------------------
    /**
     * 讀取 Follower 的當前位置與速度，計算並回傳射擊參數。
     * 若尚未呼叫 setGoal()，回傳 valid=false。
     *
     * @return ShootResult  含仰角（數學慣例）、速度、RPM
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
            if (lastValidResult != null) return lastValidResult;
            return result;
        }

        // --- A. 偏好角度優先的雙向擴散計算 ---
        ShootResult baseResult = calculatePreferredArc(
                robotX, robotY, horizontalDist, goalHeightRelative);
        if (!baseResult.valid) {
            result.errorMessage = "無法計算彈道: " + baseResult.errorMessage;
            if (lastValidResult != null) return lastValidResult;
            return result;
        }

        // --- 速度補償關閉時，直接回傳靜止解 ---
        if (!velocityCompensationEnabled) {
            lastValidResult = baseResult;
            return baseResult;
        }

        // --- B. 速度補償 ---
        // 從 Follower 取得機器人速度（Pedro Pathing 提供）
        double robotVelX   = follower.getVelocity().getXComponent();
        double robotVelY   = follower.getVelocity().getYComponent();
        double velMag      = Math.sqrt(robotVelX * robotVelX + robotVelY * robotVelY);

        // 若速度夠小，略過補償直接用靜止結果
        if (velMag < 0.5) {
            lastValidResult = baseResult;
            return baseResult;
        }

        double robotVelTheta = Math.atan2(robotVelY, robotVelX);

        ShootResult compensated = applyVelocityCompensation(
                baseResult,
                robotX, robotY,
                horizontalDist,
                goalHeightRelative,
                velMag,
                robotVelTheta,
                thetaLine
        );
        if (compensated.valid) {
            lastValidResult = compensated;
        } else if (lastValidResult != null) {
            return lastValidResult;
        }
        return compensated;
    }

    // ------------------------------------------------------------------
    // A. 偏好角度優先，雙向擴散搜索（含牆壁碰撞檢查）
    // ------------------------------------------------------------------

    /**
     * 偏好角度優先策略：
     *
     * 從 Tuning_Constant.PREFERRED_ALPHA_DEG（數學慣例）開始，同時往高角度和低角度
     * 交替擴散，找到最接近偏好角度的可行解。
     * MECH_ALPHA_MIN / MECH_ALPHA_MAX 為硬邊界，不會超過。
     *
     * 掃描順序示例（偏好 50°，step 0.5°）：
     *   50° → 50.5° → 49.5° → 51° → 49° → 51.5° → 48.5° → ...
     *
     * @param robotX 機器人場地 X (inches)，用於牆壁碰撞檢查
     * @param robotY 機器人場地 Y (inches)，用於牆壁碰撞檢查
     * @param x      水平距離 (inches)
     * @param y      目標相對高度 (inches)
     * @return       ShootResult，含最接近偏好角度的可行仰角和對應的 v0、RPM
     */
    public ShootResult calculatePreferredArc(double robotX, double robotY, double x, double y) {
        double stepRad = Math.toRadians(0.5);

        // 優先用 setPreferredAngle() 設定的值；若未設定，讀 Tuning_Constant（支援 Dashboard 即時調整）
        double preferred = (preferredAngleRad != null)
                ? preferredAngleRad
                : clamp(Math.toRadians(Tuning_Constant.PREFERRED_ALPHA_DEG), MECH_ALPHA_MIN, MECH_ALPHA_MAX);

        // 先嘗試偏好角度本身
        ShootResult r = solveForAlpha(x, y, preferred);
        if (r.valid && r.flywheelRPM <= MOTOR_MAX_RPM
                && !hitsWall(robotX, robotY, goalFieldX, goalFieldY, preferred, r.launchSpeed)) {
            return r;
        }

        // 雙向擴散：step=1 試高角度偏移，step=1 試低角度偏移，交替進行
        // 最多掃到超出 [MECH_ALPHA_MIN, MECH_ALPHA_MAX] 才停
        double maxRange = Math.max(
                preferred - MECH_ALPHA_MIN,
                MECH_ALPHA_MAX - preferred
        );
        int maxSteps = (int) Math.ceil(maxRange / stepRad) + 1;

        for (int i = 1; i <= maxSteps; i++) {
            double offset = i * stepRad;

            // 先試高角度方向
            double alphaHigh = preferred + offset;
            if (alphaHigh <= MECH_ALPHA_MAX) {
                ShootResult rHigh = solveForAlpha(x, y, alphaHigh);
                if (rHigh.valid && rHigh.flywheelRPM <= MOTOR_MAX_RPM
                        && !hitsWall(robotX, robotY, goalFieldX, goalFieldY, alphaHigh, rHigh.launchSpeed)) {
                    return rHigh;
                }
            }

            // 再試低角度方向
            double alphaLow = preferred - offset;
            if (alphaLow >= MECH_ALPHA_MIN) {
                ShootResult rLow = solveForAlpha(x, y, alphaLow);
                if (rLow.valid && rLow.flywheelRPM <= MOTOR_MAX_RPM
                        && !hitsWall(robotX, robotY, goalFieldX, goalFieldY, alphaLow, rLow.launchSpeed)) {
                    return rLow;
                }
            }

            // 若高低兩側都已超出邊界，提前結束
            if (alphaHigh > MECH_ALPHA_MAX && alphaLow < MECH_ALPHA_MIN) {
                break;
            }
        }

        ShootResult fail = new ShootResult();
        fail.errorMessage = "所有仰角均無法到達目標或會撞牆 (x=" + String.format("%.1f", x)
                + " y=" + String.format("%.1f", y)
                + " preferred=" + String.format("%.1f°", Math.toDegrees(preferred)) + ")";
        return fail;
    }

    // ------------------------------------------------------------------
    // 牆壁碰撞檢查
    // ------------------------------------------------------------------

    /**
     * 檢查從 (robotX, robotY) 射向 (goalX, goalY)、仰角 alpha（數學慣例）、初速 v0
     * 的拋射軌跡，是否會在穿越牆所在直線的位置撞到牆。
     *
     * 邏輯：
     *   1. 求射擊路徑（機器人→目標的直線）與牆直線的交點
     *   2. 若交點不在射擊路徑的範圍內（0 ~ 水平總距離），代表牆不影響這次射擊
     *   3. 若交點在範圍內，算出球在該水平距離處的拋射高度
     *   4. 若該高度低於 WALL_SAFE_HEIGHT_IN，代表會撞牆
     *
     * @return true 代表會撞牆，這個角度不可用
     */
    private boolean hitsWall(double robotX, double robotY,
                             double goalX, double goalY,
                             double alpha, double v0) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double totalDist = Math.hypot(dx, dy);
        if (totalDist < 1e-6) return false;

        double ux = dx / totalDist;
        double uy = dy / totalDist;

        // 射擊路徑與牆直線的交點（沿路徑方向的水平距離 t）
        double denom = WALL_A * ux + WALL_B * uy;
        if (Math.abs(denom) < 1e-9) {
            // 路徑與牆平行，不會交叉
            return false;
        }

        double t = -(WALL_A * robotX + WALL_B * robotY + WALL_C) / denom;

        // 交點是否落在射擊路徑範圍內
        if (t < 0 || t > totalDist) {
            return false; // 牆在路徑延伸線上，但不在機器人到目標之間
        }

        // 算出球在水平距離 t 處的高度（拋物線方程式，數學慣例 alpha）
        double cosA = Math.cos(alpha);
        double ballHeight = t * Math.tan(alpha)
                - GRAVITY * t * t / (2.0 * v0 * v0 * cosA * cosA);

        return ballHeight < WALL_SAFE_HEIGHT_IN;
    }

    // ------------------------------------------------------------------
    // B. 速度補償
    // ------------------------------------------------------------------

    private ShootResult applyVelocityCompensation(
            ShootResult base,
            double robotX,
            double robotY,
            double x,
            double y,
            double robotVelMag,
            double robotVelTheta,
            double thetaLine) {

        ShootResult result = new ShootResult();

        double v0    = base.launchSpeed;
        double alpha = base.launchAngle; // 數學慣例

        // B1：分解為徑向 / 切向
        double thetaDiff = robotVelTheta - thetaLine;
        double Vrr = -Math.cos(thetaDiff) * robotVelMag;  // 徑向（+ = 靠近目標）
        double Vrt =  Math.sin(thetaDiff) * robotVelMag;  // 切向

        // B2：飛行時間（數學慣例：cos(alpha)=水平分量比例）
        double flightTime = x / (v0 * Math.cos(alpha));
        if (flightTime <= 0) {
            result.errorMessage = "飛行時間異常";
            return result;
        }

        // B3：補償後水平速度
        double VxBase        = x / flightTime;
        double VxCompensated = VxBase + Vrr;
        double VxNew         = Math.sqrt(VxCompensated * VxCompensated + Vrt * Vrt);

        // B4：保留垂直速度（數學慣例：sin(alpha)=垂直分量比例）
        double Vy = v0 * Math.sin(alpha);

        // B5：新仰角（含 Clamp，數學慣例）
        double alphaNew = Math.atan2(Vy, VxNew);
        alphaNew = clamp(alphaNew, minAngleForComp, maxAngleForComp);

        // B6：新距離 & 新速度
        double xNew = VxNew * flightTime;
        ShootResult newSpeed = solveForAlpha(xNew, y, alphaNew);
        if (!newSpeed.valid) {
            result.errorMessage = "補償後速度計算失敗: " + newSpeed.errorMessage;
            return result;
        }

        // B6.5：補償後再檢查一次牆壁碰撞
        // 注意：補償後球的有效水平方向已經因為Vrt而偏移，
        // 這裡用補償前的目標方向線做近似檢查（保守估計）
        if (hitsWall(robotX, robotY, goalFieldX, goalFieldY, alphaNew, newSpeed.launchSpeed)) {
            result.errorMessage = "速度補償後的彈道會撞牆";
            return result;
        }

        result.launchAngle  = alphaNew;
        result.launchSpeed  = newSpeed.launchSpeed;
        result.flywheelRPM  = newSpeed.flywheelRPM;
        result.valid        = true;
        return result;
    }

    // ------------------------------------------------------------------
    // C. 已知 alpha（數學慣例），計算 v0 和 RPM
    // ------------------------------------------------------------------

    /**
     * 給定仰角 alpha（數學慣例：0°=水平，90°=垂直），計算達到 (x, y) 所需的 v0 與 RPM。
     *
     * @param x      水平距離 (inches)
     * @param y      相對高度 (inches)
     * @param alpha  仰角 (radians)，數學慣例
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

        // 關鍵保護：denom 趨近於 0（但還沒變成負數）時，v0/rpm 會像
        // 1/sqrt(denom) 一樣趨近無限大。denom<=0 的判斷抓不到這個情況，
        // 因為它在變成負數之前會先衝過天文數字。這裡用 RPM 上限
        // 直接擋住，避免回傳一個 valid=true 但數值離譜暴衝的結果。
        if (rpm > MOTOR_MAX_RPM || Double.isNaN(rpm) || Double.isInfinite(rpm)) {
            result.errorMessage = "alpha=" + formatDeg(alpha)
                    + " 需要的 RPM 過高或數值異常 (rpm=" + String.format("%.1f", rpm) + ")"
                    + "，已視為不可行角度";
            return result;
        }

        result.launchAngle  = alpha;
        result.launchSpeed  = v0;
        result.flywheelRPM  = rpm;
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
        return (launchSpeedInPerSec / getLaunchEfficiency())
                / (2.0 * Math.PI * FLYWHEEL_RADIUS_IN)
                * 60.0;
    }

    /**
     * 飛輪 RPM → 球的出膛速度 (inches/s)。
     * （方便從外部反查，例如在 Telemetry 顯示）
     */
    public static double rpmToLaunchSpeed(double rpm) {
        return rpm * (2.0 * Math.PI * FLYWHEEL_RADIUS_IN) / 60.0 * getLaunchEfficiency();
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