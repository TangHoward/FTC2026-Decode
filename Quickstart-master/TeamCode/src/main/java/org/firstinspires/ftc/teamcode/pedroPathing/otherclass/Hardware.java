package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
public class Hardware {
    // 各類物品宣告
    //直流馬達
    public DcMotorEx shooter,intake;
    //一直旋轉的伺服馬達
    public CRServo transferServo0,transferServo1, transferServo2;
    // 能夠控制角度的伺服馬達
    public Servo angleController;
    // 鏡頭
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public void init(HardwareMap hardwareMap) {
        //在設定檔 尋找直流馬達
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        // 設定直流馬達轉向
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        // 設定直流馬達是否需要使用編碼器
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //設定是否在沒有電壓時 固定/浮動
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //尋找伺服馬達
        transferServo0 = hardwareMap.get(CRServo.class, "transfer0");
        transferServo1 = hardwareMap.get(CRServo.class, "transfer1");
        transferServo2 = hardwareMap.get(CRServo.class, "transfer2");
        angleController = hardwareMap.get(Servo.class, "angleControllor");
        //設定伺服馬達轉向
        transferServo0.setDirection(CRServo.Direction.REVERSE);
        transferServo1.setDirection(CRServo.Direction.REVERSE);
        transferServo2.setDirection(CRServo.Direction.REVERSE);
        // 宣告、尋找鏡頭
        aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(true)
                        .setDrawTagID(true)
                        .setDrawCubeProjection(true)
                        .setLensIntrinsics(820.23, 820.23, 640.0, 360.0)
                        .build();

        visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .setCameraResolution(new Size(1280, 720))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .addProcessor(aprilTag)
                        .build();

    }

}
