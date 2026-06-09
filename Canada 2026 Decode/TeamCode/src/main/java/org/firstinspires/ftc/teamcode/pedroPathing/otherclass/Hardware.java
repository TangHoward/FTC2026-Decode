package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    // 各類物品宣告
    //直流馬達
    public DcMotorEx shooter0,shooter1, intake0;
    //一直旋轉的伺服馬達
    public Servo ballBlocker,angleController,turretController;

    public void init(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class,"m0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"m1");
        intake0 = hardwareMap.get(DcMotorEx.class,"m2");
        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake0.setDirection(DcMotorSimple.Direction.FORWARD);

        //turretController = hardwareMap.get(Servo.class,"turretController");
    }

}
