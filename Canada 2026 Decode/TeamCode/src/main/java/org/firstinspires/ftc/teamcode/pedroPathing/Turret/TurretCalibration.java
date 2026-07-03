package org.firstinspires.ftc.teamcode.pedroPathing.Turret;

/**
 * ============================================================
 * TurretCalibration
 * ============================================================
 *
 * GoBILDA 2000 Series 5-Turn Dual Mode Servo (25-3, Speed) 的
 * 實測校正表（用 312RPM 編碼器量測，REV/Control Hub PWM 輸出）。
 *
 * 用途：
 *   這顆伺服在 Default Mode 下 setPosition(0~1) 對應的實際角度
 *   並非簡單線性關係（尤其在低位置端有明顯跳變），
 *   因此用實測數據建立查表，搭配線性內插做雙向轉換：
 *
 *     角度 → setPosition()：呼叫 angleToPosition(angleDeg)
 *     setPosition() → 角度：呼叫 positionToAngle(position)
 *
 * 重新校正：
 *   若換新的伺服或行程有變化，重新量測後更新 CALIBRATION_TABLE，
 *   兩欄分別是 {position, actualAngleDeg}，務必依 position 遞增排序。
 *   表中角度是「伺服軸角度」，不是砲台角度。
 *
 * ── 齒輪比與砲台角度 ────────────────────────────────────────
 *   伺服 26 齒，砲台 126 齒 → 伺服轉 126/26 ≈ 4.846 圈，砲台轉 1 圈。
 *   GEAR_RATIO = 126.0 / 26.0
 *   伺服軸角度 = 砲台角度 × GEAR_RATIO
 *
 *   伺服軸總行程 1815.73°（校正表上限）換算回砲台角度：
 *   1815.73 / 4.846 ≈ 374.8°，餘裕只有約 14.8°，
 *   所以使用 turretAngleToPosition() 時務必確認輸入角度
 *   落在合理的 0°~360° 範圍，避免換算後超出伺服行程。
 *
 * ── 機構禁區（無法到達的角度區間）───────────────────────────
 *   若機構上有電線、結構等限制造成某個連續角度區間無法到達，
 *   用 setForbiddenZone() 設定（砲台角度，0°~360°，CCW+）。
 *   呼叫 turretAngleToPosition() 時，若目標落在禁區內，
 *   會自動改用最接近的邊界角度。
 * ============================================================
 */
public class TurretCalibration {

    /**
     * 實測校正表：{setPosition 值, 實際角度 (degrees)}
     * 來源：312RPM GoBILDA 編碼器實測，REV Hub Full Range Servo (500-2500µs)
     * 注意：這裡的角度是「伺服軸角度」，不是砲台角度。
     */
    private static final double[][] CALIBRATION_TABLE = {
            {0.00,    0.00},
            {0.05,  118.50},
            {0.10,  202.86},
            {0.15,  294.59},
            {0.20,  386.31},
            {0.25,  467.32},
            {0.30,  556.37},
            {0.35,  643.41},
            {0.40,  737.14},
            {0.45,  822.84},
            {0.50,  911.21},
            {0.55,  992.23},
            {0.60, 1085.96},
            {0.65, 1181.03},
            {0.70, 1267.40},
            {0.75, 1360.46},
            {0.80, 1452.85},
            {0.85, 1544.58},
            {0.90, 1632.29},
            {0.95, 1723.34},
            {1.00, 1815.73},
    };

    /** 校正表涵蓋的最小/最大「伺服軸角度」(degrees) */
    public static final double MIN_CALIBRATED_ANGLE = CALIBRATION_TABLE[0][1];
    public static final double MAX_CALIBRATED_ANGLE =
            CALIBRATION_TABLE[CALIBRATION_TABLE.length - 1][1];

    // ----------------------------------------------------------------
    // 齒輪比
    // ----------------------------------------------------------------
    /** 伺服 26 齒，砲台 126 齒：伺服轉 GEAR_RATIO 圈，砲台轉 1 圈 */
    public static final double GEAR_RATIO = 126.0 / 26.0; // ≈ 4.8462

    /** 砲台角度換算回伺服軸角度後，理論最大可達角度（供邊界檢查參考） */
    public static final double MAX_TURRET_ANGLE_DEG = MAX_CALIBRATED_ANGLE / GEAR_RATIO; // ≈ 374.8°

    /**
     * 伺服 setPosition(0) 時對應的砲台角度（degrees，0°~360°，CCW+）。
     * 你的機構：servo position=0 時，砲台在 270°。
     * 角度增加（逆時針）時，伺服軸角度也隨之增加（傳動方向一致，無反向齒輪）。
     */
    public static final double SERVO_ZERO_ANGLE_DEG = 270.0;

    // ----------------------------------------------------------------
    // 機構禁區（砲台角度，degrees，0°~360°，CCW+）
    // ----------------------------------------------------------------
    /** 禁區下限（degrees），預設無禁區（lo==hi 代表停用） */
    private static double forbiddenLo = 225;
    /** 禁區上限（degrees） */
    private static double forbiddenHi = 270;
    /** 是否啟用禁區過濾 */
    private static boolean forbiddenEnabled = true;

    /**
     * 設定機構無法到達的連續角度區間（砲台角度，0°~360°，CCW+）。
     * 例如電線或結構卡住 170°~190°：setForbiddenZone(170, 190)。
     *
     * @param loDeg 禁區下限（degrees）
     * @param hiDeg 禁區上限（degrees），須大於 loDeg
     */
    public static void setForbiddenZone(double loDeg,  double hiDeg) {
        forbiddenLo = loDeg;
        forbiddenHi = hiDeg;
        forbiddenEnabled = true;
    }

    /** 停用禁區過濾（機構上沒有限制時使用） */
    public static void clearForbiddenZone() {
        forbiddenEnabled = false;
    }

    /**
     * 檢查砲台角度是否落在禁區內。
     */
    public static boolean isInForbiddenZone(double turretAngleDeg) {
        if (!forbiddenEnabled) return false;
        return turretAngleDeg >= forbiddenLo && turretAngleDeg <= forbiddenHi;
    }

    /**
     * 若角度落在禁區內，回傳最接近的邊界角度；否則原樣回傳。
     */
    public static double filterForbiddenZone(double turretAngleDeg) {
        if (!isInForbiddenZone(turretAngleDeg)) return turretAngleDeg;

        double distToLo = Math.abs(turretAngleDeg - forbiddenLo);
        double distToHi = Math.abs(turretAngleDeg - forbiddenHi);

        return distToLo <= distToHi ? forbiddenLo : forbiddenHi;
    }

    // ----------------------------------------------------------------
    // 砲台角度 → setPosition()（含齒輪比換算 + 禁區過濾）
    // ----------------------------------------------------------------
    /**
     * 將「砲台角度」（0°~360°，CCW+）轉換成 setPosition() 需要的值。
     * 這是 TurretController 應該呼叫的主要入口方法。
     *
     * 流程：
     *   1. 若角度落在禁區內，改用最接近的邊界角度
     *   2. 減去 SERVO_ZERO_ANGLE_DEG 偏移（伺服 position=0 對應砲台 270°）
     *   3. 換算後的角度 × GEAR_RATIO → 伺服軸角度
     *   4. 查表線性內插 → setPosition() 值
     *
     * @param turretAngleDeg 目標砲台角度（degrees，0°~360°，CCW+）
     * @return setPosition() 的輸入值（0~1）
     */
    public static double turretAngleToPosition(double turretAngleDeg) {
        double filtered = filterForbiddenZone(turretAngleDeg);
        double offsetDeg = normalizeDeg(filtered - SERVO_ZERO_ANGLE_DEG);
        double servoShaftAngle = offsetDeg * GEAR_RATIO;
        return angleToPosition(servoShaftAngle);
    }

    /**
     * 將 setPosition() 的值轉換回「砲台角度」（degrees，0°~360°）。
     * 用於 telemetry 顯示目前砲台實際角度。
     *
     * @param position setPosition() 的值（0~1）
     * @return 砲台角度（degrees）
     */
    public static double positionToTurretAngle(double position) {
        double servoShaftAngle = positionToAngle(position);
        double offsetDeg = servoShaftAngle / GEAR_RATIO;
        return normalizeDeg(SERVO_ZERO_ANGLE_DEG + offsetDeg);
    }

    private static double normalizeDeg(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    // ----------------------------------------------------------------
    // 角度 → setPosition()（伺服軸角度，原始查表，內部使用）
    // ----------------------------------------------------------------
    /**
     * 將實際角度轉換成 setPosition() 需要的值（0~1），用線性內插查表。
     *
     * @param angleDeg 目標實際角度（degrees），會被 clamp 到校正範圍內
     * @return setPosition() 的輸入值（0~1）
     */
    public static double angleToPosition(double angleDeg) {
        double clamped = clamp(angleDeg, MIN_CALIBRATED_ANGLE, MAX_CALIBRATED_ANGLE);

        // 在表中找到 angleDeg 落在哪兩個校正點之間
        for (int i = 0; i < CALIBRATION_TABLE.length - 1; i++) {
            double angleLo = CALIBRATION_TABLE[i][1];
            double angleHi = CALIBRATION_TABLE[i + 1][1];

            if (clamped >= angleLo && clamped <= angleHi) {
                double posLo = CALIBRATION_TABLE[i][0];
                double posHi = CALIBRATION_TABLE[i + 1][0];

                if (angleHi == angleLo) return posLo; // 防止除零

                double t = (clamped - angleLo) / (angleHi - angleLo);
                return posLo + t * (posHi - posLo);
            }
        }

        // 理論上不會到這裡（已 clamp），保底回傳邊界值
        return clamped <= MIN_CALIBRATED_ANGLE ? 0.0 : 1.0;
    }

    // ----------------------------------------------------------------
    // setPosition() → 角度
    // ----------------------------------------------------------------
    /**
     * 將 setPosition() 的值轉換成預期實際角度，用線性內插查表。
     * 用於：已知當前 setPosition() 反推角度（例如 telemetry 顯示用）。
     *
     * @param position setPosition() 的值（0~1）
     * @return 預期實際角度（degrees）
     */
    public static double positionToAngle(double position) {
        double clamped = clamp(position, 0.0, 1.0);

        for (int i = 0; i < CALIBRATION_TABLE.length - 1; i++) {
            double posLo = CALIBRATION_TABLE[i][0];
            double posHi = CALIBRATION_TABLE[i + 1][0];

            if (clamped >= posLo && clamped <= posHi) {
                double angleLo = CALIBRATION_TABLE[i][1];
                double angleHi = CALIBRATION_TABLE[i + 1][1];

                if (posHi == posLo) return angleLo;

                double t = (clamped - posLo) / (posHi - posLo);
                return angleLo + t * (angleHi - angleLo);
            }
        }

        return clamped <= 0.0 ? CALIBRATION_TABLE[0][1]
                : CALIBRATION_TABLE[CALIBRATION_TABLE.length - 1][1];
    }

    // ----------------------------------------------------------------
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}