    package org.firstinspires.ftc.teamcode.pedroPathing.Turret;

    import com.pedropathing.follower.Follower;

    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.teamcode.pedroPathing.Shooter.LimelightLocalization;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configurable_Constant;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Hardware;
    import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

    public class TurretController {

        public enum Target {
            ID_20(20),
            ID_24(24);

            public final int tagId;

            Target(int id) {
                this.tagId = id;
            }
        }

        /**
         * 瞄準模式：
         *  APRIL_TAG - 用 Limelight AprilTag 的 tx 誤差做 PID 閉環修正
         *              （P + I + D，會持續收斂到接近 0，不是單幀 P 修正）。
         *  IMU_PID   - 用目前 Pose 算出的目標場地角度（由 aimPointX, aimPointY
         *              這個座標點反推出來，跟 APRIL_TAG 模式共用同一個
         *              fieldAngleToTarget，並非寫死角度），
         *              與 rev9AxisImu 量到的砲台實際場地朝向做 PID 閉環修正
         *              （IMU 0° = Pedro 0°，故不需要再做座標轉換）。
         */
        public enum AimMode {
            APRIL_TAG,
            IMU_PID
        }

        private final Hardware hardware;
        private final Follower follower;
        private final LimelightLocalization localization;

        private double currentAngleDeg = 90.0;
        private double targetAngleDeg = 90.0;
        private Target currentTarget = Target.ID_20;
        private double aimPointX = 16.370049;
        private double aimPointY = 130.346488;
        private double lastBaseSpeed = 0.0;
        private double lastBaseAngleRad = Math.toRadians(47.5);
        private double lastTxCorrectionDeg = 0.0;
        private boolean lastTxApplied = false;
        /** 最近一次的 Tx 原始誤差（度），= tx.txDeg - txTargetDeg，尚未經過 PID 運算，僅在 APRIL_TAG 模式且看得到 tag 時更新。 */
        private double lastTxErrorDeg = 0.0;
        private double txTargetDeg = 0.0;

        // ── 瞄準模式狀態 ──────────────────────────────────────
        private AimMode aimMode = AimMode.APRIL_TAG;

        // ── IMU PID 內部狀態 ──────────────────────────────────
        private double imuIntegralDeg = 0.0;
        private double imuLastErrorDeg = 0.0;
        private long   imuLastTimeNs = 0L;

        // ── APRIL_TAG PID 內部狀態 ────────────────────────────
        private double txIntegralDeg = 0.0;
        private double txLastErrorDeg = 0.0;
        private long   txLastTimeNs = 0L;

        /** dt 上限（秒），避免 loop 卡頓或 tag 短暫消失造成微分/積分項暴衝。 */
        private static final double MAX_DT_SEC = 0.1;

        /**
         * 死區遲滯狀態：true 代表目前正處於「凍結 I」的死區內。
         * 用兩個不同門檻（進入用 Turret_Tx_Deadband_Deg，離開用
         * Turret_Tx_Deadband_Exit_Deg，且離開門檻應大於進入門檻）
         * 避免誤差剛好卡在邊界時因雜訊反覆進出、造成 I 忽凍結忽解凍。
         */
        private boolean txInDeadband = false;
        // 拿掉這些:
// private boolean txInDeadband = false;
// (Turret_Tx_I_Fine, Turret_Tx_I_Fine_Deg, Turret_Tx_Deadband_Deg, Turret_Tx_Deadband_Exit_Deg 這些常數也不用了)

        // 新增:一個用來做 D 低通濾波的狀態,以及記錄上次拿到的原始 tx 值(判斷是否為新幀)
        private double txFilteredErrorDeg = 0.0;
        private double txLastRawTxDeg = Double.NaN;

        public TurretController(Hardware hardware, Follower follower) {
            this.hardware = hardware;
            this.follower = follower;
            this.localization = new LimelightLocalization(hardware, follower);
        }

        /**
         * 切換瞄準模式。切換時會重置 IMU PID 與 APRIL_TAG PID 的
         * 積分項/微分暫存值，避免切換瞬間因殘留誤差造成砲台角度暴衝。
         */
        public void setAimMode(AimMode mode) {
            if (aimMode == mode) return;
            aimMode = mode;
            imuIntegralDeg  = 0.0;
            imuLastErrorDeg = 0.0;
            imuLastTimeNs   = 0L;
            txIntegralDeg   = 0.0;
            txLastErrorDeg  = 0.0;
            txLastTimeNs    = 0L;
            txInDeadband    = false;
        }

        public AimMode getAimMode() {
            return aimMode;
        }

        /**
         * 切換要瞄準的 AprilTag 目標。切換時重置 tx PID 的積分項，
         * 避免舊目標殘留的誤差被錯誤套用到新目標上。
         */
        public void setTarget(Target target) {
            if (currentTarget == target) return;
            currentTarget = target;
            txIntegralDeg  = 0.0;
            txLastErrorDeg = 0.0;
            txLastTimeNs   = 0L;
            txInDeadband   = false;
        }

        /**
         * 設定要瞄準的場地座標點（inches，Pedro 座標系）。
         * 不論目前是哪一種瞄準模式，都是瞄準這個座標點——
         * APRIL_TAG 模式用視覺 tx PID 修正，IMU_PID 模式用 IMU 角度
         * 做 PID 閉環，兩者的「目標角度」都是由這個座標點即時算出來的
         * fieldAngleToTarget，並非寫死角度。
         * <p>
         * 若座標點跳動幅度較大（例如切換瞄準目標），會重置 IMU PID
         * 與 APRIL_TAG PID 的積分項，避免殘留誤差造成暴衝。
         */
        public void setAimPoint(double fieldX, double fieldY) {
            double jump = Math.hypot(fieldX - aimPointX, fieldY - aimPointY);
            if (jump > 6.0) {
                imuIntegralDeg  = 0.0;
                imuLastErrorDeg = 0.0;
                imuLastTimeNs   = 0L;
                txIntegralDeg   = 0.0;
                txLastErrorDeg  = 0.0;
                txLastTimeNs    = 0L;
//                txInDeadband    = false;
            }
            aimPointX = fieldX;
            aimPointY = fieldY;
        }

        public double getAimPointX() {
            return aimPointX;
        }

        public double getAimPointY() {
            return aimPointY;
        }

        public void setTxTarget(double targetDeg) {
            txTargetDeg = targetDeg;
        }

        public double getTxTarget() {
            return txTargetDeg;
        }

        /**
         * 用 Limelight 重新定位 Follower（更新 Pedro Pose）。
         * <p>
         * LimelightLocalization.update() 要的參數是
         * 「砲台絕對場地朝向」（turretLocalAngleRad，這裡實際傳入
         * rev9AxisImu 量到的 yaw + offset，跟 IMU_PID 瞄準模式用同一顆
         * IMU、同樣的定義），update() 內部會自己拿這個角度去餵給 MT2，
         * 並在讀回結果後做品質過濾（角速度、距離、時序一致性等）。
         * <p>
         * 融合改用 fuseToPedroAdaptive()：依 tag 數與距離自動決定要
         * 相信視覺多少，並對單次融合的位移量/角度做限幅，避免機器人
         * 因為視覺讀值跟目前估計差距較大就被瞬間「拉」過去。
         */
        public boolean relocalizeWithLimelight() {
            double turretGobalRad = Math.toRadians(hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);
            LimelightLocalization.LLPose pose = localization.update(turretGobalRad);
            if (pose.valid) {
                return localization.fuseToPedroAdaptive();
            }
            return false;
        }

        /**
         * 「停止重新定位」流程用：呼叫端（Control_Mode）應該在呼叫這個方法的
         * 同時把搖桿輸入歸零、讓機器人真正停下來。LimelightLocalization.update()
         * 內部的平移/角速度閘控會確認底盤跟砲台都夠靜止才採用讀值，通過時序
         * 一致性檢查後，才用 fuseToPedroStationary()（不做保守限幅）做修正。
         * <p>
         * 建議用法：driver 按住某個鍵時，每個 loop 都呼叫這個方法，直到
         * getLimelightRejectReason() 回傳 "OK" 或放開按鍵為止。
         */
        public boolean relocalizeStationary() {
            double turretGobalRad = Math.toRadians(hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);
            LimelightLocalization.LLPose pose = localization.update(turretGobalRad);
            if (pose.valid) {
                return localization.fuseToPedroStationary();
            }
            return false;
        }

        /** 除錯用：回傳最近一次視覺定位被拒絕/採用的原因，方便比賽現場看 telemetry 除錯。 */
        public String getLimelightRejectReason() {
            return localization.getLastRejectReason();
        }

        public double getTurretAngularVelocityDegPerSec() {
            return localization.getTurretAngularVelocityDegPerSec();
        }

        public double getChassisAngularVelocityDegPerSec() {
            return localization.getChassisAngularVelocityDegPerSec();
        }

        /** 一般用法：正常瞄準，不凍結砲台。 */
        public void update() {
            update(false, false);
        }

        /**
         * @param freezeAim 為 true 時（例如「停止重新定位」流程中），完全不
         *                  重新計算/移動砲台角度，只維持在目前位置：
         *                  1) 讓 sampleMotion() 估計出的砲台角速度盡快收斂到 0，
         *                     不會因為砲台還在微調而一直被角速度閘控擋掉；
         *                  2) 相機視角在這幾幀之間保持穩定，有利於視覺讀值一致性；
         *                  3) 順便重置 IMU / Tx PID 的計時（不清積分值），避免
         *                     解凍後第一次 update() 因為 dt 暴增（凍結了好幾幀）
         *                     造成微分項暴衝——這跟 setAimMode() 切換時的處理
         *                     邏輯一致。
         */
        public void update(boolean freezeAim, boolean shooting) {
            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRad = follower.getPose().getHeading();

            // 每個 loop 都取樣一次目前的砲台角度／底盤朝向，供
            // LimelightLocalization 估計角速度以做動態品質過濾。
            // 用「上一輪」的 currentAngleDeg（尚未被本輪覆寫）取樣即可，
            // 跟實際呼叫間隔（每個 loop 都呼叫）比對起來誤差可忽略。
            localization.sampleMotion(Math.toRadians(currentAngleDeg), robotHeadingRad);

            if (freezeAim) {
                // 凍結中：不改 currentAngleDeg、不呼叫 setPosition()，
                // 砲台維持在目前角度不動。只重置計時，避免解凍後 dt 暴衝。
                imuLastTimeNs = 0L;
                txLastTimeNs  = 0L;
                lastTxCorrectionDeg = 0.0;
                lastTxApplied = false;
                return;
            }

            double turretCX = robotX
                    + LimelightLocalization.TURRET_TO_ROBOT_IN
                    * Math.cos(robotHeadingRad + Math.PI);
            double turretCY = robotY
                    + LimelightLocalization.TURRET_TO_ROBOT_IN
                    * Math.sin(robotHeadingRad + Math.PI);

            double dx = aimPointX - turretCX;
            double dy = aimPointY - turretCY;
            double fieldAngleToTarget = Math.atan2(dy, dx);

            double rawDeg = Math.toDegrees(fieldAngleToTarget - robotHeadingRad) + 90.0;
            targetAngleDeg = normalizeAngle(rawDeg);

            double velX = follower.getVelocity().getXComponent();
            double velY = follower.getVelocity().getYComponent();
            double velMag = Math.sqrt(velX * velX + velY * velY);

            // follower.getVelocity() 是 field-centric 的「平移」速度分量，
            // 不包含原地旋轉的角速度，所以拿它當作「底盤是否在平移」的判斷
            // 完全符合「原地轉頭瞄準時仍允許積分累加」的需求。
            boolean isTranslating = velMag > Tuning_Constant.Turret_Tx_I_Freeze_Speed_InPerSec;

            double turretOffsetDeg = 0.0;
            if (velMag > 0.5 && lastBaseSpeed > 0.0) {
                double thetaVel = Math.atan2(velY, velX);
                double thetaDiff = thetaVel - fieldAngleToTarget;
                double vTangential = Math.sin(thetaDiff) * velMag;
                double vRadial = -Math.cos(thetaDiff) * velMag;

                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist > 1.0) {
                    double baseVx = lastBaseSpeed * Math.cos(lastBaseAngleRad);
                    double flightTime = dist / Math.max(baseVx + vRadial, 1.0);
                    double lateralOffset = vTangential * flightTime;
                    turretOffsetDeg = Math.toDegrees(Math.atan2(lateralOffset, dist));
                }
            }

            currentAngleDeg = normalizeAngle(targetAngleDeg + turretOffsetDeg);

            if (aimMode == AimMode.APRIL_TAG) {
                LimelightLocalization.TxResult tx = localization.getTxForTag(currentTarget.tagId);
                if (tx.valid) {
                    double txError = tx.txDeg - txTargetDeg;
                    lastTxErrorDeg = txError;

                    long nowNs = System.nanoTime();
                    double dt = (txLastTimeNs == 0L) ? 0.0 : (nowNs - txLastTimeNs) / 1e9;
                    dt = clamp(dt, 0.0, MAX_DT_SEC);
                    txLastTimeNs = nowNs;

                    // ── D:只在真的拿到「新一幀」視覺讀值時才計算微分，
                    //     並先對誤差做一階低通濾波，避免雜訊被微分放大 ──
                    boolean isNewFrame = Double.isNaN(txLastRawTxDeg) || tx.txDeg != txLastRawTxDeg;
                    txLastRawTxDeg = tx.txDeg;

                    double derivativeDegPerSec = 0.0;
                    if (isNewFrame && dt > 1e-6) {
                        double alpha = 0.3; // 低通濾波係數，可調
                        txFilteredErrorDeg = txFilteredErrorDeg + alpha * (txError - txFilteredErrorDeg);
                        derivativeDegPerSec = (txFilteredErrorDeg - txLastErrorDeg) / dt;
                        txLastErrorDeg = txFilteredErrorDeg;
                    }
                    // 不是新幀時，derivative 直接視為 0 貢獻（不是拿舊數字硬算），
                    // 避免同一幀被重複當成「誤差沒變化」而錯誤地拉低微分項。

                    // ── P：每輪都用當下 error 直接算，不做任何條件式跳過 ──
                    // ── 依誤差大小選擇要用「一般」或「精細」PID 增益 ──
                    boolean useFineGains = Math.abs(txError) < Tuning_Constant.Turret_Tx_Fine_Threshold_Deg;
                    double kP = useFineGains ? Tuning_Constant.Turret_Tx_P_Fine : Tuning_Constant.Turret_Tx_P;
                    double kI = useFineGains ? Tuning_Constant.Turret_Tx_I_Fine : Tuning_Constant.Turret_Tx_I;
                    double kD = useFineGains ? Tuning_Constant.Turret_Tx_D_Fine : Tuning_Constant.Turret_Tx_D;

                    // ── P：每輪都用當下 error 直接算，不做任何條件式跳過 ──
                    double pOut = txError * kP;

                    // ── I：先算出「假設不限幅」的 output，用來做 anti-windup 判斷 ──
                    double iOutRaw = (txIntegralDeg + txError * dt) * kI;
                    double dOut = derivativeDegPerSec * kD;
                    double pidOutputDegRaw = pOut + iOutRaw + dOut;
                    double pidOutputDeg = clamp(pidOutputDegRaw,
                            -Tuning_Constant.Turret_Tx_MAX_CORR, Tuning_Constant.Turret_Tx_MAX_CORR);

                    boolean saturated = (pidOutputDegRaw != pidOutputDeg);
                    boolean errorPushesAwayFromSaturation =
                            (pidOutputDeg > 0 && txError < 0) || (pidOutputDeg < 0 && txError > 0);

                    // Anti-windup：只要沒有飽和，或飽和但誤差正把輸出往回拉，就允許 I 累積。
                    // 拿掉「isTranslating 就整個凍結」這種二元條件——移動造成的暫態誤差
                    // 應該透過調整 dt 上限 / Ki 大小來處理，而不是整段跳過積分。
                    if (!saturated || errorPushesAwayFromSaturation) {
                        txIntegralDeg += txError * dt;
                        txIntegralDeg = clamp(txIntegralDeg,
                                -Tuning_Constant.Turret_Tx_I_MAX, Tuning_Constant.Turret_Tx_I_MAX);
                    }

                    currentAngleDeg = normalizeAngle(currentAngleDeg - pidOutputDeg);
                    lastTxCorrectionDeg = -pidOutputDeg;
                    lastTxApplied = true;

                } else {
                    // tag 短暫看不到：只重置 dt 計時，保留 txIntegralDeg 不清零
                    txLastTimeNs = 0L;
                    txLastRawTxDeg = Double.NaN; // 下次看到 tag 時強制視為新幀，避免用到過期的 dt/derivative
                    lastTxCorrectionDeg = 0.0;
                    lastTxApplied = false;
                }
            } else { // AimMode.IMU_PID
                // 量測：rev9AxisImu 的 yaw，其 0° 已對齊 Pedro 的 0°（全域座標），
                // 可以直接跟 fieldAngleToTarget 比較，不需要再套用任何座標轉換。
                double measuredFieldHeadingRad =
                        Math.toRadians(hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);

                // 關鍵修正：PID 比對用的目標角必須是「加上 turretOffsetDeg（PDF Part B 前導角補償）後」的角度，
                // 不能用原始 fieldAngleToTarget。否則穩態時 PID 會把砲台實際朝向拉回沒補償的角度，
                // 等於把前導角補償吃掉。turretOffsetDeg 是純角度平移量，field 座標系跟 servo 座標系
                // 之間只差一個 robotHeading 旋轉 + 90° 平移（線性關係），所以可以直接加回 fieldAngleToTarget。
                double compensatedFieldAngleRad = fieldAngleToTarget + Math.toRadians(turretOffsetDeg);

                double errorRad = normalizeSignedRad(compensatedFieldAngleRad - measuredFieldHeadingRad);
                double errorDeg = Math.toDegrees(errorRad);

                long nowNs = System.nanoTime();
                double dt = (imuLastTimeNs == 0L) ? 0.0 : (nowNs - imuLastTimeNs) / 1e9;
                dt = clamp(dt, 0.0, MAX_DT_SEC);
                imuLastTimeNs = nowNs;

                imuIntegralDeg += errorDeg * dt;
                imuIntegralDeg = clamp(imuIntegralDeg,
                        -Tuning_Constant.Turret_IMU_I_MAX, Tuning_Constant.Turret_IMU_I_MAX);

                double derivativeDegPerSec = (dt > 1e-6) ? (errorDeg - imuLastErrorDeg) / dt : 0.0;
                imuLastErrorDeg = errorDeg;

                double pidOutputDeg =
                        errorDeg            * Tuning_Constant.Turret_IMU_P
                                + imuIntegralDeg      * Tuning_Constant.Turret_IMU_I
                                + derivativeDegPerSec * Tuning_Constant.Turret_IMU_D;

                currentAngleDeg = normalizeAngle(currentAngleDeg + pidOutputDeg);

                lastTxCorrectionDeg = pidOutputDeg;
                lastTxApplied = true;
            }

            double servoPosition = TurretCalibration.turretAngleToPosition(currentAngleDeg);

            if(!shooting){
            hardware.turretController.setPosition(servoPosition);
            }
        }

        public void setAngleDirect(double angleDeg) {
            currentAngleDeg = normalizeAngle(angleDeg);
            targetAngleDeg = currentAngleDeg;
            double servoPosition = TurretCalibration.turretAngleToPosition(currentAngleDeg);
            hardware.turretController.setPosition(servoPosition);
        }

        public double getCurrentAngleDeg() {
            return currentAngleDeg;
        }

        public double getTargetAngleDeg() {
            return targetAngleDeg;
        }

        public Target getCurrentTarget() {
            return currentTarget;
        }

        public LimelightLocalization getLocalization() {
            return localization;
        }

        public double getLastTxCorrectionDeg() {
            return lastTxCorrectionDeg;
        }

        /**
         * 取得最近一次的 Tx 原始誤差（度），= 視覺讀到的 tx - txTargetDeg，
         * 尚未經過 PID 運算（跟 getLastTxCorrectionDeg() 回傳的「PID 輸出修正量」不同）。
         * 只有在 APRIL_TAG 模式且看得到 tag 時才會更新；tag 消失時維持上一次的值，
         * 不會跳回 0，方便 telemetry 判斷「最後一次量到的誤差有多大」。
         */
        public double getLastTxErrorDeg() {
            return lastTxErrorDeg;
        }

        public boolean isLastTxApplied() {
            return lastTxApplied;
        }

        public void setLastBaseSpeed(double speedInPerSec) {
            if (speedInPerSec > 0.0) lastBaseSpeed = speedInPerSec;
        }

        public void setLastBaseAngle(double angleRad) {
            if (angleRad > 0.0 && !Double.isNaN(angleRad) && !Double.isInfinite(angleRad)) {
                lastBaseAngleRad = angleRad;
            }
        }

        public boolean isTargetInForbiddenZone() {
            return TurretCalibration.isInForbiddenZone(targetAngleDeg);
        }

        private static double normalizeAngle(double deg) {
            deg = deg % 360.0;
            if (deg < 0) deg += 360.0;
            return deg;
        }

        /** 把弧度誤差正規化到 [-π, π]，避免 359°→1° 被誤判成繞一大圈 */
        private static double normalizeSignedRad(double rad) {
            rad = rad % (2 * Math.PI);
            if (rad > Math.PI)  rad -= 2 * Math.PI;
            if (rad < -Math.PI) rad += 2 * Math.PI;
            return rad;
        }

        private static double clamp(double v, double min, double max) {
            return Math.max(min, Math.min(max, v));
        }
    }