package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;
import org.firstinspires.ftc.teamcode.Subsytem.OpmodeStorage;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{

    private Arm arm=null;
    private Intake intake=null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        waitForStart();

        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();



        onStop();

    }

    public void onStop(){
        OpmodeStorage.slidePos = arm.getSlideLenght();
    }
}