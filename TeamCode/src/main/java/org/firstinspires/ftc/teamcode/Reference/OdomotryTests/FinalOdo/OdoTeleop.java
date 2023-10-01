
package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.FinalOdo;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Reference.OdomotryTests.FinalOdo.OdometrySystem;

@TeleOp(name="OdoTeleop", group="Iterative Opmode")

public class OdoTeleop extends OpMode {

    public OdometrySystem thread;

    public void init() {

        thread = new OdometrySystem(hardwareMap);
        thread.start();

    }

    public void loop(){

        // Put Teleop Code Here

    }

    @Override
    public void stop() {
        thread.interrupt();
    }
}
