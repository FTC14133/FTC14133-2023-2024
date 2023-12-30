
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Subsystems.GetPos;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{

    private GetPos getPos=null;
    private SampleMecanumDrive drive=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        getPos = new GetPos(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        while (!opModeIsActive() && !isStopRequested()){

            getPos.returnAprilPos(telemetry, drive, gamepad1);
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

    }
}