
package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.ThreadingTest;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ThreadingAuto", group="Auto")

public class ThreadingAuto extends LinearOpMode {

    public LoopTest thread;

    public void HardwareStart() {

        thread = new LoopTest(hardwareMap, telemetry);
        thread.start();

    }

    public void runOpMode(){

        waitForStart();
        HardwareStart();


        telemetry.addData("Status", "On");
        telemetry.update();

        try {
            Thread.sleep(3500);
        } catch (Exception e) {
            return;
        }

        telemetry.addData("Status", "Off");
        telemetry.update();

        try {
            Thread.sleep(3500);
        } catch (Exception e) {
            return;
        }
        endThread();
    }

    public void endThread() {
        thread.interrupt();
        super.stop();
    }
}
