package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public DcMotorEx shooter,intake;
    public CRServo transferServo0,transferServo1, transferServo2;
    public Servo angleController;
    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        transferServo0 = hardwareMap.get(CRServo.class, "transfer0");
        transferServo1 = hardwareMap.get(CRServo.class, "transfer1");
        transferServo2 = hardwareMap.get(CRServo.class, "transfer2");
        angleController = hardwareMap.get(Servo.class, "angleControllor");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //陳晉嘉要改下面那排 改成shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        transferServo0.setDirection(CRServo.Direction.REVERSE);
        transferServo1.setDirection(CRServo.Direction.REVERSE);
        transferServo2.setDirection(CRServo.Direction.REVERSE);

    }

}
