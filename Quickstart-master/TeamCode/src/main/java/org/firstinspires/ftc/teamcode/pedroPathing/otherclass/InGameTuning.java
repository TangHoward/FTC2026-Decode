package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class InGameTuning {
    // 實時調整 (如何使用請看Configurable_Constants)
    // 機器瞄準的誤差 遠、近 (數值越大越往場中瞄準)
    public static double longLunchBallXError = 0;
    public static double nearLunchBallXError = 0;
    // 手動 遠、近 誤差 (數值越大 拋越高)
    public static double longLunchAngleError = 1;
    public static double nearLunchAngleError = 6;
}
