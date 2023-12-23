
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Subsystems.GetPos;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{

    private GetPos getPos=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        getPos = new GetPos(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        while (!opModeIsActive() && !isStopRequested()){
            getPos.returnAprilPos(telemetry, 4);
            telemetry.addData("e", "e");
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }
}