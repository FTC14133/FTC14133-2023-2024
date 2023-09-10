
package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.FinalOdo;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Reference.OdomotryTests.FinalOdo.OdometrySystem;

@Autonomous(name="ThreadingAuto", group="Auto")

public class OdoAuto extends LinearOpMode {

    public OdometrySystem thread;

    public void HardwareStart() {

        thread = new OdometrySystem(hardwareMap);
        thread.start();

    }

    public void runOpMode(){

        // Put Autonomous Code Here

        endThread(); // Put This At The End
    }

    public void endThread() {
        thread.interrupt();
        super.stop();
    }
}
