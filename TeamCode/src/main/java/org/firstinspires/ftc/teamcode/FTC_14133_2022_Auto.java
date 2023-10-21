package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsytem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsytem.Odometry;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{

    private Drivetrain drivetrain=null;
    private Odometry odometry=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        drivetrain = new Drivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        waitForStart();

        telemetry.addData("Object", "Passed waitForStart");

        drivetrain.GoToCoord(5, 5, 0.5, odometry, telemetry);

        telemetry.addData("pos", odometry.Return_Coords());
        telemetry.update();
        sleep(10000);
    }
}