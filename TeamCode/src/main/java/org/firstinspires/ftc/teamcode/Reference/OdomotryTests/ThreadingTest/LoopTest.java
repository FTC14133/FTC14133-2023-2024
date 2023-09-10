package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.ThreadingTest;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTest extends Thread{

    public Telemetry telemetry;
    public boolean loop = true;

    private static DcMotorEx lf;

    public LoopTest(HardwareMap hardwareMap, Telemetry privateTelemetry){

        telemetry = privateTelemetry;

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lf.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public void run(){

        while (!(Thread.currentThread().isInterrupted())) {

            lf.setPower(0.25);

            try {
                Thread.sleep(1000);
            } catch (Exception e){return;}

            lf.setPower(0.15);

            try {
                Thread.sleep(1000);
            } catch (Exception e){return;}


        }
    }
}
