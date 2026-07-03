package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    // 各類物品宣告
    //直流馬達
    public DcMotorEx shooter0,shooter1, intake0,intake1;

    public Servo angleController,blocker,turretController;
    public Limelight3A limelight;
    public Rev9AxisImu rev9AxisImu;
    public void init(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        intake0 = hardwareMap.get(DcMotorEx.class,"intake0");
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake0.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        angleController = hardwareMap.get(Servo.class, "angleController");
        turretController = hardwareMap.get(Servo.class,"turretController");
        blocker = hardwareMap.get(Servo.class,"blocker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rev9AxisImu = hardwareMap.get(Rev9AxisImu.class, "revIMU");
        angleController.setDirection(Servo.Direction.REVERSE);
        turretController.setDirection(Servo.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.FORWARD);

    }

}
