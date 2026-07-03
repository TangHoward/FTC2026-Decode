package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

/**
 * ============================================================
 * LimelightLocalization  (修正版)
 * ============================================================
 *
 * 透過 Limelight MegaTag2 botpose 計算機器人在 Pedro 場地座標的 Pose，
 * 並將結果回寫給 Pedro Follower，輔助定位。
 *
 * ── 座標系統 ────────────────────────────────────────────────
 *  Limelight botpose : 原點 = 場地中心，單位 meters，Yaw CCW+
 *  Pedro             : 原點 = 場地左下角，單位 inches，Heading CCW+
 *
 *  位置轉換（X/Y 互換 + 取負）：
 *      pedro_x = ll_y_in + FIELD_CX
 *      pedro_y = -ll_x_in + FIELD_CY
 *  這在數學上等價於「LL 座標系相對 Pedro 座標系旋轉了 -90°」
 *  （轉換矩陣 [[0,1],[-1,0]] = R(-90°)，是純旋轉、無鏡射）。
 *
 *  ★★★ 重要修正 ★★★
 *  既然位置轉換是一個 -90° 的旋轉，角度（heading/yaw）也必須套用
 *  「同一個」旋轉才會自洽，因此：
 *      pedro_heading = ll_yaw - HEADING_OFFSET_DEG   （讀回）
 *      ll_yaw_to_feed = pedro_heading + HEADING_OFFSET_DEG  （餵入 MT2）
 *  預設 HEADING_OFFSET_DEG = 90，YAW_SIGN = 1（純旋轉理論值）。
 *
 *  MegaTag2 的 yaw 是求解器的輸入約束之一，會直接影響求出的 X/Y！
 *  （不只是拿來讀值而已）因此 updateRobotOrientation() 餵入的角度
 *  「必須」跟這裡的座標系一致，不能直接丟 Pedro heading 原始值。
 *
 *  ⚠ 以上 HEADING_OFFSET_DEG / YAW_SIGN 是根據座標轉換矩陣反推出的
 *  理論值，實際數值可能因 Limelight 安裝方向或場地設定而不同。
 *  建議實測校正：把機器人轉到已知角度（0°/90°/180°/270°），
 *  比對 Pedro heading 與這裡算出的 measuredFieldHeadingRad，
 *  再微調這兩個常數。
 *
 * ── 機構幾何補償 ────────────────────────────────────────────
 *  砲台為伺服位置模式，相機裝在砲台上、會獨立於底盤旋轉。
 *
 *  ★★★ 重要修正 ★★★
 *  MT2 的 updateRobotOrientation() 需要的是「相機（砲台）當下的
 *  場地朝向」，不是底盤朝向（因為 Limelight 端沒有另外設定隨砲台
 *  即時更新的 camera-robot-space offset，等於把相機視為機器人本身）。
 *  因此 update() 現在改成接收「砲台相對底盤的本地角度」
 *  （turretLocalAngleRad），內部用：
 *      砲台場地朝向估計 = 底盤朝向估計(follower.getHeading())
 *                        + (砲台本地角度 - 90°)
 *  組出來餵給 MT2；MT2 讀回的 yaw 代表砲台場地朝向，
 *  再用同一個關係反推回底盤朝向，才寫回 result.robotHeading，
 *  避免把砲台朝向誤當底盤朝向污染 Pedro 的姿態估計。
 *
 *  CAM_TO_TURRET 補償使用「MT2 讀回的砲台場地朝向」；
 *  TURRET_TO_ROBOT 補償使用「反推出來的底盤朝向」——
 *  因為砲台圓心到機器中心的機構偏移是固定在底盤上的，
 *  跟砲台目前轉到哪個角度無關。
 *
 * ── 使用方式 ────────────────────────────────────────────────
 *  LimelightLocalization loc = new LimelightLocalization(hardware, follower);
 *
 *  // 每個 loop（或需要重新定位時）呼叫：
 *  double turretLocalAngleRad =
 *          Math.toRadians(TurretCalibration.positionToTurretAngle(currentServoPosition));
 *  LimelightLocalization.LLPose p = loc.update(turretLocalAngleRad);
 *  if (p.valid) {
 *      loc.fuseToPedro(p, 0.7); // 回寫融合到 Pedro Follower
 *  }
 * ============================================================
 */
public class LimelightLocalization {

    // ── 場地常數（FTC DECODE，Pedro 座標系）─────────────
    /** FTC DECODE 場地：6×6 tiles，每 tile 24"，共 144"×144" */
    private static final double FIELD_LEN_IN = 144.0;
    private static final double FIELD_WID_IN = 144.0;
    public  static final double FIELD_CX     = FIELD_LEN_IN / 2.0; // 72.0"
    public  static final double FIELD_CY     = FIELD_WID_IN / 2.0; // 72.0"

    // ── 機構偏移（inches）────────────────────────────────
    /** Limelight 距砲台圓心前方距離（沿砲台朝向） */
    public static final double CAM_TO_TURRET_IN   = 173.0 / 25.4; // 6.811"
    /** 砲台圓心距機器中心距離（沿機器後方） */
    public static final double TURRET_TO_ROBOT_IN = 29.5  / 25.4; // 1.161"

    // ── Heading / Yaw 轉換常數 ────────────────────────────
    /**
     * LL yaw → Pedro heading 的角度偏移量（度）。
     * 由座標轉換矩陣 [[0,1],[-1,0]] 反推為 -90° 旋轉，
     * 故 pedro_heading = ll_yaw - HEADING_OFFSET_DEG。
     * ★ 務必實測校正，此為理論推導值，非實測值。
     */
    public static final double HEADING_OFFSET_DEG = 90;

    /**
     * 額外的方向修正（1.0 或 -1.0）。
     * 若實測發現角度變化方向相反（機器人左轉但讀值變小），改為 -1.0。
     */
    public static final double YAW_SIGN = 1.0;

    /**
     * 砲台「本地角度」定義中，機器人正前方對應的角度值（degrees）。
     * 依 TurretController 慣例：0°=機器人右方，90°=機器人正前方，CCW+。
     * 這裡拿來把「砲台本地角度」轉換成跟 Pedro heading 同一個參考方向
     * （Pedro heading 0° 慣例上代表機器人正前方）。
     * ★ 如果你的 TurretController 定義有改，這裡要同步改。
     */
    public static final double TURRET_LOCAL_FORWARD_OFFSET_DEG = 0;
    private static final double TURRET_LOCAL_FORWARD_OFFSET_RAD =
            Math.toRadians(TURRET_LOCAL_FORWARD_OFFSET_DEG);

    // ── 有效性過濾 ────────────────────────────────────────
    public static final long   MAX_STALENESS_MS = 50L;
    public static final double MAX_JUMP_IN      = 18.0;
    public static final int    MIN_TAG_COUNT    = 1;

    // ── 結果容器 ──────────────────────────────────────────
    public static class LLPose {
        public double  robotX       = 0;
        public double  robotY       = 0;
        public double  robotHeading = 0;
        public long    stalenessMs  = 0;
        public int     tagCount     = 0;
        public boolean valid        = false;
    }

    // ── 內部狀態 ──────────────────────────────────────────
    private final Hardware  hardware;
    private final Follower  follower;
    private double  lastX         = FIELD_CX;
    private double  lastY         = FIELD_CY;
    private boolean hasFirstValid = false;

    // ── 建構子 ────────────────────────────────────────────
    public LimelightLocalization(Hardware hardware, Follower follower) {
        this.hardware = hardware;
        this.follower = follower;
        hardware.limelight.pipelineSwitch(0);
        hardware.limelight.start();
    }

    /**
     * 把 Pedro heading（弧度，CCW+，0=+X）轉成餵給
     * updateRobotOrientation() 用的 LL 座標系角度（度）。
     * 跟 llYawToPedroHeading() 互為反函數。
     */
    private static double pedroHeadingToLLYawDeg(double pedroHeadingRad) {
        double pedroDeg = Math.toDegrees(pedroHeadingRad);
        return (pedroDeg + HEADING_OFFSET_DEG) / YAW_SIGN;
    }

    /** LL yaw（度）→ Pedro heading（弧度），跟上面互為反函數。 */
    private static double llYawToPedroHeadingRad(double llYawDeg) {
        return Math.toRadians(llYawDeg * YAW_SIGN - HEADING_OFFSET_DEG);
    }

    // ── 主要更新 ──────────────────────────────────────────
    /**
     * @param turretGobleAngleRad 砲台「相對底盤」的目前角度（radians）。
     *                            依 TurretController 慣例：0°=機器人右方，
     *                            90°=機器人正前方，CCW+。
     *                            建議直接用
     *                            Math.toRadians(TurretCalibration.positionToTurretAngle(currentServoPosition))
     *                            取得，確保跟實際伺服位置同步。
     */
    public LLPose update(double turretGobleAngleRad) {
        // ★修正：相機是裝在「會獨立旋轉的砲台」上，不是固定在底盤，
        // 所以 MT2 需要的是「砲台（相機）當下的場地朝向」，不是底盤朝向。
        // 用「目前底盤朝向估計 + 砲台本地角度」組出砲台的場地朝向估計，
        // 再轉換到 LL 座標系後餵給 updateRobotOrientation()。
        double turretFieldHeadingEstimateRad = normalizeRad(turretGobleAngleRad - TURRET_LOCAL_FORWARD_OFFSET_RAD);

        double llYawToFeedDeg = pedroHeadingToLLYawDeg(turretFieldHeadingEstimateRad);
        hardware.limelight.updateRobotOrientation(llYawToFeedDeg);

        LLPose result = new LLPose();

        LLResult llResult = hardware.limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return result;

        long staleness = llResult.getStaleness();
        if (staleness > MAX_STALENESS_MS) return result;

        int tagCount = llResult.getBotposeTagCount();
        if (tagCount < MIN_TAG_COUNT) return result;

        Pose3D botpose = llResult.getBotpose_MT2();
        if (botpose == null) return result;

        double camX_m = botpose.getPosition().toUnit(DistanceUnit.METER).x;
        double camY_m = botpose.getPosition().toUnit(DistanceUnit.METER).y;
        double yawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

        // ── 座標轉換 ─────────────────────────────────────
        // botpose 原點 = 場地中心；LL 的 -X 軸 → Pedro +Y 軸，
        // LL 的 +Y 軸 → Pedro +X 軸。
        double camY = camX_m * 39.3701 * -1 + FIELD_CY;
        double camX = camY_m * 39.3701 * 1 + FIELD_CX;

        // ── Heading 轉換（★修正：套用跟位置轉換一致的 -90° 旋轉）──
        // MT2 回傳的 yaw 代表「砲台（相機）的場地朝向」，不是底盤朝向。
        double measuredTurretFieldHeadingRad = llYawToPedroHeadingRad(Math.toRadians(llYawToFeedDeg));

        // 補償：相機 → 砲台圓心（沿「砲台」場地朝向退回）
        double turretCX = camX - CAM_TO_TURRET_IN * Math.cos(measuredTurretFieldHeadingRad);
        double turretCY = camY - CAM_TO_TURRET_IN * Math.sin(measuredTurretFieldHeadingRad);

        double robotX = turretCX + TURRET_TO_ROBOT_IN * Math.cos(follower.getHeading());
        double robotY = turretCY + TURRET_TO_ROBOT_IN * Math.sin(follower.getHeading());

        if (hasFirstValid) {
            double jump = Math.hypot(robotX - lastX, robotY - lastY);
            if (jump > MAX_JUMP_IN) return result;
        }

        if (robotX < -12 || robotX > FIELD_LEN_IN + 12
                || robotY < -12 || robotY > FIELD_WID_IN + 12) return result;

        lastX         = robotX;
        lastY         = robotY;
        hasFirstValid = true;

        result.robotX       = robotX;
        result.robotY       = robotY;
        result.robotHeading = normalizeRad(follower.getHeading());
        result.stalenessMs  = staleness;
        result.tagCount     = tagCount;
        result.valid        = true;
        latestValidPose     = result;
        return result;
    }

    // ── 快取最新有效 Pose ─────────────────────────────────
    private LLPose latestValidPose = null;

    public LLPose getLatestValidPose() { return latestValidPose; }

    public boolean fuseLatestToPedro(double pedroWeight) {
        if (latestValidPose == null) return false;
        fuseToPedro(latestValidPose, pedroWeight);
        return true;
    }

    public void fuseToPedro(LLPose pose, double pedroWeight) {
        if (!pose.valid) return;
        Pose current = follower.getPose();

        double fusedX = current.getX() * pedroWeight + pose.robotX * (1 - pedroWeight);
        double fusedY = current.getY() * pedroWeight + pose.robotY * (1 - pedroWeight);

        // ★修正：角度融合改用圓形平均（circular mean），避免在 0°/360°
        // 邊界附近出現「350° 跟 10° 平均成 180°」這種錯誤結果。
        double headingErr = normalizeRad(pose.robotHeading - current.getHeading());
        if (headingErr > Math.PI) headingErr = headingErr - 2 * Math.PI; // 取最短路徑（-π ~ π）

        double fusedHeading;
        if (Math.abs(headingErr) > Math.toRadians(10)) {
            // 用 pedroWeight 統一控制 X/Y/heading 的融合權重
            fusedHeading = normalizeRad(current.getHeading() + headingErr * (1 - pedroWeight));
        } else {
            fusedHeading = current.getHeading();
        }

        follower.setPose(new Pose(fusedX, fusedY, fusedHeading));
    }

    public void resetFilter(double x, double y) {
        lastX         = x;
        lastY         = y;
        hasFirstValid = true;
    }

    // ── 即時 tx 查詢 ──────────────────────────────────────
    public TxResult getTxForTag(int tagId) {
        TxResult result = new TxResult();

        LLResult llResult = hardware.limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return result;

        long staleness = llResult.getStaleness();
        if (staleness > MAX_STALENESS_MS) return result;

        java.util.List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        if (fiducials == null) return result;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == tagId) {
                result.txDeg = f.getTargetXDegrees();
                result.valid = true;
                return result;
            }
        }
        return result;
    }

    public static class TxResult {
        public double  txDeg = 0.0;
        public boolean valid = false;
    }

    private static double normalizeRad(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < 0) rad += 2 * Math.PI;
        return rad;
    }
}