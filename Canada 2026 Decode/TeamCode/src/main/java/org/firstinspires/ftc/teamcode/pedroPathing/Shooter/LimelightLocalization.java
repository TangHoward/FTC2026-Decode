    package org.firstinspires.ftc.teamcode.pedroPathing.Shooter;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.Pose;

    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

    import com.qualcomm.hardware.limelightvision.LLResult;
    import com.qualcomm.hardware.limelightvision.LLResultTypes;

    /**
     * ============================================================
     * LimelightLocalization  (修正版 + 品質過濾 / 漸進式融合版)
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
     *  MT2 的 updateRobotOrientation() 需要的是「相機（砲台）當下的
     *  場地朝向」，不是底盤朝向（因為 Limelight 端沒有另外設定隨砲台
     *  即時更新的 camera-robot-space offset，等於把相機視為機器人本身）。
     *
     *  update(turretGobleAngleRad) 的參數，實際呼叫端
     *  （TurretController.relocalizeWithLimelight()）目前傳入的是
     *  rev9AxisImu 量到的「砲台絕對場地朝向」（Math.toRadians(yaw + offset)），
     *  這跟 TurretController 的 IMU_PID 瞄準模式用同一顆 IMU、用同一種方式
     *  （直接拿 yaw 當砲台場地朝向、不再加底盤朝向）互相一致。
     *  ★ 因此這裡「不需要」再把底盤朝向加回去——turretGobleAngleRad
     *  本身就已經是場地角度了。若之後改成傳入「砲台相對底盤的本地角度」，
     *  才需要在這裡加上 follower.getHeading()，屆時務必同步更新本註解
     *  與 update() 內的計算，否則會把底盤朝向重複疊加，造成方向計算錯誤。
     *
     *  CAM_TO_TURRET 補償使用「MT2 讀回的砲台場地朝向」；
     *  TURRET_TO_ROBOT 補償使用「反推出來的底盤朝向」——
     *  因為砲台圓心到機器中心的機構偏移是固定在底盤上的，
     *  跟砲台目前轉到哪個角度無關。
     *
     * ── 動態品質過濾（新增）────────────────────────────────────
     *  單純的 staleness / tagCount / MAX_JUMP_IN 過濾不足以應付：
     *    1) 砲台或底盤轉動過快時的動態模糊 / IMU-相機時間不同步
     *    2) 遠距離單一 tag 造成的三角測量雜訊
     *    3) 單幀雜訊造成的偶發性錯誤讀值（但還沒大到觸發 MAX_JUMP_IN）
     *  因此加入：
     *    - 角速度閘控：sampleMotion() 由 TurretController.update()
     *      每個 loop 呼叫，獨立估計砲台 / 底盤角速度（deg/s）。
     *      角速度過高時，update() 直接拒絕本次視覺讀值。
     *      （角速度用「每個 loop 都取樣」而非「每次 update() 呼叫」估計，
     *      是因為 update() 只在需要重定位時才呼叫，取樣間隔不固定，
     *      直接拿它的間隔算角速度會嚴重失真。）
     *    - 距離信心度：用 LLResult.getBotposeAvgDist()（官方 API，回傳本次
     *      botpose 計算所用 tag 的平均距離，公尺）換算成信心度，距離越遠、
     *      tag 數越少，信心度越低，融合時的視覺權重也越低。
     *    - 時序一致性檢查：連續兩幀（在 CONSISTENCY_WINDOW_MS 內）的候選
     *      位置必須互相吻合（差距 <= CONSISTENCY_TOLERANCE_IN）才會被採用，
     *      濾掉單幀雜訊造成的偶發跳動，同時不會像永久拒絕那樣擋住真正的
     *      漂移修正（只要下一幀讀值穩定重現，就會被接受）。
     *    - 底盤平移速度閘控：底盤還在移動（超過
     *      Vision_Max_Chassis_LinearVel_InS）時直接拒收，避免移動模糊
     *      與視覺延遲造成的位置落後被誤採用。
     *    - Tag 合法性過濾：畫面中只要出現不在 VALID_LOCALIZATION_TAG_IDS
     *      清單裡的 tag（例如 DECODE 場地的 Obelisk 21~23 號），整次
     *      botpose 讀值就不採用，避免位置不保證準確的 tag 汙染 MT2 求解。
     *
     * ── 停止重新定位（建議的主要使用情境）──────────────────────
     *  比起「邊跑邊修」，更穩健的做法是讓機器人主動停下來（搖桿輸入歸零）、
     *  確認底盤/砲台角速度與平移速度都低於閾值後，再連續幾幀採樣視覺讀值，
     *  通過時序一致性確認後，用 fuseToPedroStationary() 做較大幅度的修正——
     *  因為機器人本來就沒在動、沒有在跑路徑，不需要 fuseToPedroAdaptive()
     *  那種怕造成「瞬間平移觀感」的保守限幅，可以直接修正到位、收斂更快。
     *  呼叫端範例見 TurretController.relocalizeStationary()。
     *
     * ── 漸進式融合（新增）──────────────────────────────────────
     *  就算視覺讀值通過以上所有品質過濾，Pedro 目前估計的位置跟視覺讀值
     *  之間仍可能存在較大落差（例如剛開機、或前面漂移累積了一段時間）。
     *  若直接依照融合權重整個跳過去，機器人在路徑跟隨中會被感覺到「瞬間平移」。
     *  fuseToPedro() / fuseToPedroAdaptive() 因此對每次融合的位移量與角度
     *  修正量都做了限幅（Vision_Max_Correction_Per_Fuse_In /
     *  Vision_Max_Heading_Correction_Deg），大幅落差會分成好幾個 loop
     *  逐漸收斂，而不是一次到位，效果類似指數平滑（exponential smoothing）。
     *
     * ── 使用方式 ────────────────────────────────────────────────
     *  LimelightLocalization loc = new LimelightLocalization(hardware, follower);
     *
     *  // 每個 loop 都呼叫（cheap，只是取樣角度做角速度估計）：
     *  loc.sampleMotion(turretLocalOrFieldAngleRad, follower.getPose().getHeading());
     *
     *  // 需要重定位時才呼叫（例如按下按鍵）：
     *  LimelightLocalization.LLPose p = loc.update(turretFieldAngleRad);
     *  if (p.valid) {
     *      loc.fuseToPedroAdaptive(); // 依信心度自動決定融合權重 + 漸進式限幅
     *  }
     *  // 除錯用：loc.getLastRejectReason() 會回傳本次拒絕/採用的原因字串
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
         * 砲台角度參數中，機器人正前方對應的角度值（degrees）。
         * 目前 update() 收到的角度是「砲台絕對場地朝向」（見類別註解），
         * 這個常數保留給未來若改成傳入「砲台本地角度」時使用；
         * 目前預設 0，代表不做額外偏移。
         */
        public static final double TURRET_LOCAL_FORWARD_OFFSET_DEG = 0;
        private static final double TURRET_LOCAL_FORWARD_OFFSET_RAD =
                Math.toRadians(TURRET_LOCAL_FORWARD_OFFSET_DEG);

        // ── 有效性過濾 ────────────────────────────────────────
        public static final long   MAX_STALENESS_MS = 50L;
        public static final double MAX_JUMP_IN      = 18.0;
        public static final int    MIN_TAG_COUNT    = 1;

        /**
         * 合法可用於定位的 AprilTag ID（FTC DECODE 賽季：20=藍方 Goal，
         * 24=紅方 Goal）。場地中央 Obelisk 上的 21~23 號 tag 位置不保證
         * 準確，不能拿來定位，畫面中只要出現不在這個清單裡的 tag，
         * 整次 botpose 讀值就不採用（因為 MT2 的求解是把畫面中看到的
         * tag 一起算，混進一顆位置不準的 tag 會把結果一起帶歪）。
         * ★ 每季場地佈局都不同，換賽季務必更新這個清單。
         */
        private static final java.util.Set<Integer> VALID_LOCALIZATION_TAG_IDS =
                new java.util.HashSet<>(java.util.Arrays.asList(20, 24));

        // ── 結果容器 ──────────────────────────────────────────
        public static class LLPose {
            public double  robotX       = 0;
            public double  robotY       = 0;
            public double  robotHeading = 0;
            public long    stalenessMs  = 0;
            public int     tagCount     = 0;
            /** 0~1，依 tag 數與平均距離估計的可信度，供漸進式融合當作視覺權重 */
            public double  confidence   = 0;
            public boolean valid        = false;
        }

        // ── 內部狀態 ──────────────────────────────────────────
        private final Hardware  hardware;
        private final Follower  follower;
        private double  lastX         = FIELD_CX;
        private double  lastY         = FIELD_CY;
        private boolean hasFirstValid = false;

        /** 最近一次 update() 的結果說明（除錯／telemetry 用） */
        private String lastRejectReason = "NONE";

        // ── 動作取樣狀態（角速度估計，供 sampleMotion() 使用）───
        private double lastSampledTurretAngleRad   = Double.NaN;
        private double lastSampledChassisHeadingRad = Double.NaN;
        private long   lastSampleTimeNs = 0L;
        private double turretAngularVelDegS  = 0.0;
        private double chassisAngularVelDegS = 0.0;

        // ── 時序一致性緩衝（連續兩幀需互相吻合才採信）──────────
        private double  pendingX = 0, pendingY = 0;
        private long    pendingTimeMs = 0L;
        private boolean pendingValid = false;

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

        // ── 動作取樣（角速度閘控用，每個 loop 呼叫）────────────
        /**
         * 每個 loop 呼叫一次（不論本次是否要做視覺重定位），
         * 用來獨立估計砲台與底盤的角速度。跟 update() 分開，
         * 是因為 update() 只有需要重定位時才呼叫（例如按鍵觸發），
         * 呼叫間隔不固定，若拿 update() 自己的呼叫間隔去算角速度，
         * 會嚴重低估／高估真實角速度。
         *
         * @param turretAngleRad   砲台目前角度（跟餵給 update() 的角度
         *                         用同一種定義即可，這裡只在意變化量）
         * @param chassisHeadingRad 底盤目前 heading（follower.getPose().getHeading()）
         */
        public void sampleMotion(double turretAngleRad, double chassisHeadingRad) {
            long nowNs = System.nanoTime();
            if (lastSampleTimeNs != 0L && !Double.isNaN(lastSampledTurretAngleRad)) {
                double dt = (nowNs - lastSampleTimeNs) / 1e9;
                if (dt > 1e-4) {
                    double turretDeltaRad = normalizeSignedRad(turretAngleRad - lastSampledTurretAngleRad);
                    turretAngularVelDegS = Math.abs(Math.toDegrees(turretDeltaRad) / dt);

                    double chassisDeltaRad = normalizeSignedRad(chassisHeadingRad - lastSampledChassisHeadingRad);
                    chassisAngularVelDegS = Math.abs(Math.toDegrees(chassisDeltaRad) / dt);
                }
            }
            lastSampledTurretAngleRad    = turretAngleRad;
            lastSampledChassisHeadingRad = chassisHeadingRad;
            lastSampleTimeNs = nowNs;
        }

        public double getTurretAngularVelocityDegPerSec()  { return turretAngularVelDegS; }
        public double getChassisAngularVelocityDegPerSec() { return chassisAngularVelDegS; }
        public String getLastRejectReason() { return lastRejectReason; }

        // ── 可信度估計 ────────────────────────────────────────
        /**
         * 依 tag 數與平均距離估計本次讀值的可信度（0~1）。
         * tag 數越多、距離越近，可信度越高；用來決定漸進式融合時
         * 該相信視覺多少（視覺權重），而不是每次都用固定權重。
         */
        private static double computeConfidence(int tagCount, double avgDistIn) {
            double tagFactor = tagCount >= 2 ? 1.0 : 0.55;

            double nearIn = 48.0; // 4 英尺內視為近距離，滿信心
            double farIn  = Math.max(nearIn + 1.0, Tuning_Constant.Vision_Max_Trust_Dist_In);

            double distFactor;
            if (avgDistIn <= nearIn) {
                distFactor = 1.0;
            } else {
                distFactor = 1.0 - (avgDistIn - nearIn) / (farIn - nearIn);
            }
            distFactor = clamp(distFactor, 0.2, 1.0);

            return clamp(tagFactor * distFactor, 0.15, 1.0);
        }

        // ── 主要更新 ──────────────────────────────────────────
        /**
         * @param turretGobleAngleRad 砲台「絕對場地朝向」（radians）。
         *                            目前由 TurretController.relocalizeWithLimelight()
         *                            傳入 rev9AxisImu 量到的 yaw + offset，
         *                            跟 IMU_PID 瞄準模式用同一顆 IMU、同樣直接
         *                            當場地角度使用，兩處定義必須保持一致。
         */
        public LLPose update(double turretGobleAngleRad) {
            double turretFieldHeadingEstimateRad = normalizeRad(turretGobleAngleRad - TURRET_LOCAL_FORWARD_OFFSET_RAD);

            double llYawToFeedDeg = pedroHeadingToLLYawDeg(turretFieldHeadingEstimateRad);
            // MegaTag2 每一幀都需要餵朝向（會直接影響求解出的 X/Y），
            // 所以這裡「一定」要呼叫，即使等一下角速度過高會拒絕這次結果。
            hardware.limelight.updateRobotOrientation(llYawToFeedDeg);

            LLPose result = new LLPose();

            // ── 品質過濾 1：角速度過高（動態模糊 / IMU-相機不同步風險）──
            if (turretAngularVelDegS > Tuning_Constant.Vision_Max_Turret_AngVel_DegS) {
                lastRejectReason = "TURRET_TOO_FAST(" + String.format("%.0f", turretAngularVelDegS) + "dps)";
                return result;
            }
            if (chassisAngularVelDegS > Tuning_Constant.Vision_Max_Chassis_AngVel_DegS) {
                lastRejectReason = "CHASSIS_TOO_FAST(" + String.format("%.0f", chassisAngularVelDegS) + "dps)";
                return result;
            }

            // ── 品質過濾：底盤平移速度過快（移動模糊 + 視覺延遲造成位置落後）──
            // Follower 本身就有速度資訊，不需要額外取樣估計。
            double chassisLinVelIn = Math.hypot(
                    follower.getVelocity().getXComponent(),
                    follower.getVelocity().getYComponent());
            if (chassisLinVelIn > Tuning_Constant.Vision_Max_Chassis_LinearVel_InS) {
                lastRejectReason = "CHASSIS_MOVING(" + String.format("%.1f", chassisLinVelIn) + "in/s)";
                return result;
            }

            LLResult llResult = hardware.limelight.getLatestResult();
            if (llResult == null || !llResult.isValid()) {
                lastRejectReason = "NO_RESULT";
                return result;
            }

            long staleness = llResult.getStaleness();
            if (staleness > MAX_STALENESS_MS) {
                lastRejectReason = "STALE(" + staleness + "ms)";
                return result;
            }

            int tagCount = llResult.getBotposeTagCount();
            if (tagCount < MIN_TAG_COUNT) {
                lastRejectReason = "NO_TAG";
                return result;
            }

            // ── 品質過濾：畫面中混入不合法的 tag（例如 DECODE 的 Obelisk）──
            // MT2 的 botpose 是把畫面中看到的 tag 一起求解，只要有一顆
            // 位置不保證準確的 tag 混進來，整次結果都不能信任。
            java.util.List<LLResultTypes.FiducialResult> fiducialsInView = llResult.getFiducialResults();
            if (fiducialsInView != null) {
                for (LLResultTypes.FiducialResult f : fiducialsInView) {
                    if (!VALID_LOCALIZATION_TAG_IDS.contains(f.getFiducialId())) {
                        lastRejectReason = "INVALID_TAG_IN_VIEW(" + f.getFiducialId() + ")";
                        return result;
                    }
                }
            }

            Pose3D botpose = llResult.getBotpose_MT2();
            if (botpose == null) {
                lastRejectReason = "NULL_BOTPOSE";
                return result;
            }

            // ── 品質過濾 2：平均距離過遠（三角測量雜訊過大）──────
            double avgDistIn = llResult.getBotposeAvgDist() * 39.3701;
            if (avgDistIn > Tuning_Constant.Vision_Max_Trust_Dist_In) {
                lastRejectReason = "TOO_FAR(" + String.format("%.0f", avgDistIn) + "in)";
                return result;
            }

            double camX_m = botpose.getPosition().toUnit(DistanceUnit.METER).x;
            double camY_m = botpose.getPosition().toUnit(DistanceUnit.METER).y;

            // ── 座標轉換 ─────────────────────────────────────
            // botpose 原點 = 場地中心；LL 的 -X 軸 → Pedro +Y 軸，
            // LL 的 +Y 軸 → Pedro +X 軸。
            double camY = camX_m * 39.3701 * -1 + FIELD_CY;
            double camX = camY_m * 39.3701 * 1 + FIELD_CX;

            // MT2 回傳的 yaw 代表「砲台（相機）的場地朝向」，不是底盤朝向。
            double measuredTurretFieldHeadingRad = llYawToPedroHeadingRad(Math.toRadians(llYawToFeedDeg));

            // 補償：相機 → 砲台圓心（沿「砲台」場地朝向退回）
            double turretCX = camX - CAM_TO_TURRET_IN * Math.cos(measuredTurretFieldHeadingRad);
            double turretCY = camY - CAM_TO_TURRET_IN * Math.sin(measuredTurretFieldHeadingRad);

            double robotX = turretCX + TURRET_TO_ROBOT_IN * Math.cos(follower.getHeading());
            double robotY = turretCY + TURRET_TO_ROBOT_IN * Math.sin(follower.getHeading());

            // ── 品質過濾 3：跟上次採用的位置差距過大（硬性防線）────
            if (hasFirstValid) {
                double jump = Math.hypot(robotX - lastX, robotY - lastY);
                if (jump > MAX_JUMP_IN) {
                    lastRejectReason = "JUMP_TOO_LARGE(" + String.format("%.1f", jump) + "in)";
                    return result;
                }
            }

            if (robotX < -12 || robotX > FIELD_LEN_IN + 12
                    || robotY < -12 || robotY > FIELD_WID_IN + 12) {
                lastRejectReason = "OUT_OF_BOUNDS";
                return result;
            }

            // ── 品質過濾 4：時序一致性檢查 ─────────────────────
            // 連續兩幀的候選位置要互相吻合才採信，濾掉單幀雜訊造成的
            // 偶發跳動；但只要下一幀讀值穩定重現，就會被接受，
            // 不會永久擋住真正的漂移修正。
            long nowMs = System.currentTimeMillis();
            boolean consistent = pendingValid
                    && (nowMs - pendingTimeMs) <= Tuning_Constant.Vision_Consistency_Window_Ms
                    && Math.hypot(robotX - pendingX, robotY - pendingY) <= Tuning_Constant.Vision_Consistency_Tolerance_In;

            pendingX = robotX;
            pendingY = robotY;
            pendingTimeMs = nowMs;
            pendingValid = true;

            if (!consistent) {
                lastRejectReason = "PENDING_CONFIRM";
                return result; // 先不採用，等下一幀確認過再融合
            }

            lastX = robotX;
            lastY = robotY;
            hasFirstValid = true;

            result.robotX       = robotX;
            result.robotY       = robotY;
            // 目前刻意讓 heading 直接沿用 Pedro 現有估計（等同於 heading 融合
            // 為 no-op）：因為 rev9AxisImu 已經提供可靠的絕對朝向，這也是
            // Limelight 官方文件在有可靠陀螺儀時的建議做法──視覺只修正位置，
            // 朝向繼續信任 IMU（stdev 設很大）。若之後想改用視覺修正朝向，
            // 這裡才需要換成 measuredTurretFieldHeadingRad 反推底盤朝向。
            result.robotHeading = normalizeRad(follower.getHeading());
            result.stalenessMs  = staleness;
            result.tagCount     = tagCount;
            result.confidence   = computeConfidence(tagCount, avgDistIn);
            result.valid        = true;
            lastRejectReason    = "OK";
            latestValidPose      = result;
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

        /**
         * 依可信度（tag 數 / 距離）自動決定視覺權重並融合，
         * 同時套用漸進式限幅，是建議的主要進入點。
         */
        public boolean fuseToPedroAdaptive() {
            if (latestValidPose == null || !latestValidPose.valid) return false;
            fuseInternal(latestValidPose, latestValidPose.confidence);
            return true;
        }

        /** 保留原本的固定權重介面（向下相容），內部一樣會套用漸進式限幅。 */
        public void fuseToPedro(LLPose pose, double pedroWeight) {
            if (!pose.valid) return;
            fuseInternal(pose, 1.0 - pedroWeight);
        }

        /**
         * 專門給「停止重新定位」流程用：呼叫端已經把機器人停下來
         * （搖桿輸入歸零、平移速度閘控也已確保底盤沒在動），
         * 這種情況下不需要 fuseInternal() 那種怕路徑跟隨被瞬間拉走的
         * 保守限幅——機器人本來就沒在動、也沒有在跑路徑，位置直接
         * 修正到位反而更快更準（呼應 Iron Reign 文章裡的
         * "Hard Reset" 策略：靜止時可以直接覆寫，不會有軌跡不連續的問題）。
         * <p>
         * 權重仍然依可信度（tag 數 / 距離）決定，並上限在 0.95，
         * 保留一點點對 Pedro 既有估計的信任，避免單幀離群值直接覆蓋。
         * heading 沿用底盤 IMU（與一般融合邏輯一致，見 update() 內註解）。
         */
        public boolean fuseToPedroStationary() {
            if (latestValidPose == null || !latestValidPose.valid) return false;
            LLPose pose = latestValidPose;
            double visionWeight = clamp(pose.confidence, 0.0, 0.95);
            Pose current = follower.getPose();

            double fusedX = current.getX() * (1 - visionWeight) + pose.robotX * visionWeight;
            double fusedY = current.getY() * (1 - visionWeight) + pose.robotY * visionWeight;

            follower.setPose(new Pose(fusedX, fusedY, current.getHeading()));
            return true;
        }

        /**
         * 實際融合邏輯：
         *   1. 先算出「完全依權重融合」會落在哪裡（targetX/targetY）
         *   2. 若這個目標點離目前 Pedro 位置太遠，限制單次最大位移量
         *      （Vision_Max_Correction_Per_Fuse_In），分成多個 loop 逐漸收斂，
         *      避免機器人在路徑跟隨中被瞬間「拉」過去。
         *   3. heading 沿用圓形平均（避免 0°/360° 邊界問題）+ 10° 死區
         *      （小誤差不修正，信任 IMU）+ 同樣做角度修正量限幅。
         */
        private void fuseInternal(LLPose pose, double visionWeight) {
            visionWeight = clamp(visionWeight, 0.0, Tuning_Constant.Vision_Max_Fuse_Weight);
            Pose current = follower.getPose();

            double targetX = current.getX() * (1 - visionWeight) + pose.robotX * visionWeight;
            double targetY = current.getY() * (1 - visionWeight) + pose.robotY * visionWeight;

            double dist = Math.hypot(targetX - current.getX(), targetY - current.getY());
            double maxStep = Tuning_Constant.Vision_Max_Correction_Per_Fuse_In;

            double fusedX, fusedY;
            if (dist > maxStep && dist > 1e-6) {
                double scale = maxStep / dist;
                fusedX = current.getX() + (targetX - current.getX()) * scale;
                fusedY = current.getY() + (targetY - current.getY()) * scale;
            } else {
                fusedX = targetX;
                fusedY = targetY;
            }

            // ★ 角度融合改用圓形平均（circular mean），避免在 0°/360°
            // 邊界附近出現「350° 跟 10° 平均成 180°」這種錯誤結果。
            double headingErr = normalizeRad(pose.robotHeading - current.getHeading());
            if (headingErr > Math.PI) headingErr = headingErr - 2 * Math.PI; // 取最短路徑（-π ~ π）

            double fusedHeading;
            if (Math.abs(headingErr) > Math.toRadians(10)) {
                double step = headingErr * visionWeight;
                double maxHeadingStep = Math.toRadians(Tuning_Constant.Vision_Max_Heading_Correction_Deg);
                step = clamp(step, -maxHeadingStep, maxHeadingStep);
                fusedHeading = normalizeRad(current.getHeading() + step);
            } else {
                fusedHeading = current.getHeading();
            }

            follower.setPose(new Pose(fusedX, fusedY, fusedHeading));
        }

        public void resetFilter(double x, double y) {
            lastX         = x;
            lastY         = y;
            hasFirstValid = true;
            pendingValid  = false;
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

        /** 把弧度誤差正規化到 [-π, π]，避免 359°→1° 被誤判成繞一大圈 */
        private static double normalizeSignedRad(double rad) {
            rad = rad % (2 * Math.PI);
            if (rad > Math.PI)  rad -= 2 * Math.PI;
            if (rad < -Math.PI) rad += 2 * Math.PI;
            return rad;
        }

        private static double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }
    }