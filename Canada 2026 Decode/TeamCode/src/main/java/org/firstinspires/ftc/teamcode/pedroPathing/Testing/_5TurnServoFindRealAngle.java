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

@TeleOp(group = "5TurnServo",name = "_5TurnServoFindOutput")
public class _5TurnServoFindRealAngle extends LinearOpMode {

    private Servo s0;
    private DcMotor m0;
    private double[] tableX = new double[41];
    private final double[] tableY = {
            0.00,45,90.00,135, 180.00,225, 270.00,315, 360.00,405, 450.00,495, 540.00,585, 630.00,675, 720.00,765, 810.00,
            855, 900.00,945, 990.00,1035, 1080.00,1125, 1170.00,1215, 1260.00,1305, 1350.00,1395, 1440.00,1485, 1530.00,1575, 1620.00,
            1665,1710.00,1755, 1800.00
    };
    private TelemetryManager telemetryM;
    private _5TurnServoRegulate turnServoRegulate;
    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        String Text,OutPut;
        int i;
        s0 = hardwareMap.get(Servo.class, "s0");
        m0 = hardwareMap.get(DcMotor.class, "m0");

        Text = "";
        OutPut ="";
        m0.setDirection(DcMotorSimple.Direction.REVERSE);
        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            s0.setPosition(0);
            sleep(3250);
            m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            for (i = 0; i <= 40; i += 1) {
                s0.setPosition(Math.max(0, Math.min(1, (double) i/40)));
                sleep(300);
                OutPut = OutPut + String.format(Locale.US,"%.2f, ",m0.getCurrentPosition()/537.7*360);
                tableX[i] =m0.getCurrentPosition()/537.7*360;

            }
            telemetryM.debug("WithoutRegulationErrorAverage",errorAverage(tableX,tableY));
            telemetryM.debug("tableOutPut","{" +OutPut.substring(0,OutPut.length()-2)+"}");

            s0.setPosition(0);
            sleep(3250);
            turnServoRegulate = new _5TurnServoRegulate(tableX,tableY);
            OutPut ="";
            for (i = 0; i <= 40; i += 1) {
                s0.setPosition(Math.max(0, Math.min(1, turnServoRegulate.regulate((double)i/40))));
                sleep(300);
                OutPut = OutPut + String.format(Locale.US,"%.2f, ",m0.getCurrentPosition()/537.7*360);
                tableX[i] =m0.getCurrentPosition()/537.7*360;

            }
            telemetryM.debug("WithRegulationErrorAverage",errorAverage(tableX,tableY));
            telemetryM.debug("RegulatedOutput","{" +OutPut.substring(0,OutPut.length()-2)+"}" );
            telemetryM.update();
            while (opModeIsActive()){

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

