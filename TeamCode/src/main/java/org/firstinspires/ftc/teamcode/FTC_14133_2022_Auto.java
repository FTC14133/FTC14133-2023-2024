
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetection;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{

    private AprilTagDetection aprilTagDetection=null;

    public void HardwareStart() throws InterruptedException {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        aprilTagDetection = new AprilTagDetection(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode() throws InterruptedException {

        HardwareStart();

        waitForStart();

        telemetry.addData("Object", "Passed waitForStart");

        aprilTagDetection.goToTag(telemetry, 4);

        telemetry.update();

    }
}