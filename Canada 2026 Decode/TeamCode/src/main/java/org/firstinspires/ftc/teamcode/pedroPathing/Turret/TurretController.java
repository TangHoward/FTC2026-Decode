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
     * 「砲台相對底盤的本地角度」（turretLocalAngleRad，
     * 0°=機器右方，90°=機器前方，CCW+），也就是 currentAngleDeg
     * 本身，不需要（也不應該）先換算成場地朝向——
     * update() 內部會自己拿 follower.getHeading() 跟這個角度組合，
     * 算出砲台的場地朝向去餵給 MT2，並在讀回結果後正確反推底盤朝向。
     */
    public boolean relocalizeWithLimelight() {
        double turretGobalRad = Math.toRadians(hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);
        LimelightLocalization.LLPose pose = localization.update(turretGobalRad);
        if (pose.valid) {
            localization.fuseToPedro(pose, 0.5);
            return true;
        }
        return false;
    }

    public void update() {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = follower.getPose().getHeading();

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

                long nowNs = System.nanoTime();
                double dt = (txLastTimeNs == 0L) ? 0.0 : (nowNs - txLastTimeNs) / 1e9;
                txLastTimeNs = nowNs;

                txIntegralDeg += txError * dt;
                txIntegralDeg = clamp(txIntegralDeg,
                        -Tuning_Constant.Turret_Tx_I_MAX, Tuning_Constant.Turret_Tx_I_MAX);

                double derivativeDegPerSec = (dt > 1e-6) ? (txError - txLastErrorDeg) / dt : 0.0;
                txLastErrorDeg = txError;

                double pidOutputDeg =
                        txError               * Tuning_Constant.Turret_Tx_P
                                + txIntegralDeg       * Tuning_Constant.Turret_Tx_I
                                + derivativeDegPerSec * Tuning_Constant.Turret_Tx_D;

                // 限幅，避免大誤差時單幀修正量暴衝
                pidOutputDeg = clamp(pidOutputDeg,
                        -Tuning_Constant.Turret_Tx_MAX_CORR, Tuning_Constant.Turret_Tx_MAX_CORR);

                currentAngleDeg = normalizeAngle(currentAngleDeg - pidOutputDeg);
                lastTxCorrectionDeg = -pidOutputDeg;
                lastTxApplied = true;
            } else {
                // tag 短暫看不到時，只重置 dt 計時避免下次 dt 突然暴增造成微分項暴衝，
                // 但保留 txIntegralDeg，避免每次 tag 閃爍就把累積的修正歸零。
                txLastTimeNs = 0L;
                lastTxCorrectionDeg = 0.0;
                lastTxApplied = false;
            }
        } else { // AimMode.IMU_PID
            // 量測：rev9AxisImu 的 yaw，其 0° 已對齊 Pedro 的 0°（全域座標），
            // 可以直接跟 fieldAngleToTarget 比較，不需要再套用任何座標轉換。
            double measuredFieldHeadingRad =
                    Math.toRadians(hardware.rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + Configurable_Constant.turretAngleOffset);

            double errorRad = normalizeSignedRad(fieldAngleToTarget - measuredFieldHeadingRad);
            double errorDeg = Math.toDegrees(errorRad);

            long nowNs = System.nanoTime();
            double dt = (imuLastTimeNs == 0L) ? 0.0 : (nowNs - imuLastTimeNs) / 1e9;
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

            // 沿用同一組 telemetry 欄位，方便顯示端不用改
            lastTxCorrectionDeg = pidOutputDeg;
            lastTxApplied = true;
        }

        double servoPosition = TurretCalibration.turretAngleToPosition(currentAngleDeg);
        hardware.turretController.setPosition(servoPosition);
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