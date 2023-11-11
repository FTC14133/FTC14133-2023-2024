package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{


    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();


        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        waitForStart();

        telemetry.addData("Object", "Passed waitForStart");


        telemetry.update();

    }
}