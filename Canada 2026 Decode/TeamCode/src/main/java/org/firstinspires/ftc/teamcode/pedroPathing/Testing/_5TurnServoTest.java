package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing._5TurnServo._5TurnServoRegulate;

import java.util.Locale;

//@TeleOp(group = "5TurnServo", name = "_5TurnServoTest")
public class _5TurnServoTest  extends LinearOpMode {
    private Servo s0;
    private DcMotor m0;
    private double[] tableX = {0.00, 60.93, 109.13, 151.98, 195.50, 241.03, 288.56, 331.41, 373.59, 417.78, 465.98, 506.16, 549.01, 593.19, 642.07, 686.93, 731.11, 777.98, 814.13, 858.99, 901.84, 946.03, 988.21, 1031.06, 1080.60, 1124.12, 1173.00, 1219.86, 1262.04, 1311.59, 1357.78, 1396.62, 1450.18, 1494.36, 1534.54, 1583.41, 1624.25, 1669.11, 1719.32, 1766.19, 1809.71};
    private final double[] tableY = {
            0.00,    40.50,   81.00,   121.50,  162.00,  202.50,  243.00,  283.50,
            324.00,  364.50,  405.00,  445.50,  486.00,  526.50,  567.00,  607.50,
            648.00,  688.50,  729.00,  769.50,  810.00,  850.50,  891.00,  931.50,
            972.00,  1012.50, 1053.00, 1093.50, 1134.00, 1174.50, 1215.00, 1255.50,
            1296.00, 1336.50, 1377.00, 1417.50, 1458.00, 1498.50, 1539.00, 1579.50,
            1620.00
    };
    private TelemetryManager telemetryM;
    private _5TurnServoRegulate turnServoRegulate;

    @Override
    public void runOpMode(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        String Text,OutPut;
        int i;
        s0 = hardwareMap.get(Servo.class, "s0");
        m0 = hardwareMap.get(DcMotor.class, "m0");

        Text = "";
        OutPut ="";
        m0.setDirection(DcMotorSimple.Direction.REVERSE);
        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnServoRegulate = new _5TurnServoRegulate(tableX,tableY);
        waitForStart();
        if(opModeIsActive()) {
            s0.setPosition(0);
            sleep(3250);
            m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            for (i = 0; i <= 40; i += 1) {
                s0.setPosition(Math.max(0, Math.min(1, turnServoRegulate.regulate((double)i/40))));
                sleep(300);
                OutPut = OutPut + String.format(Locale.US,"%.2f, ",m0.getCurrentPosition()/537.7*360);
                tableX[i] =m0.getCurrentPosition()/537.7*360;

            }
            telemetryM.debug("WithoutRegulationErrorAverage",errorAverage(tableX,tableY));
            telemetryM.debug("tableOutPut","{" +OutPut.substring(0,OutPut.length()-2)+"}");
            telemetryM.update();
            while (opModeIsActive()) {

            }
        }
    }
    private double errorAverage(double[] tableX, double[] tableY){
        double sum = 0;
        for (int i = 0; i < tableX.length; i++){
            sum += Math.abs(tableX[i] -tableY[i]);
        }
        return sum/tableX.length;
    }
}
