package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Tuning_Constant;

@TeleOp
public class ServoTest extends OpMode {
    private Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"s0");
    }

    @Override
    public void loop() {
        servo.setPosition(Tuning_Constant.servo);
    }
}
