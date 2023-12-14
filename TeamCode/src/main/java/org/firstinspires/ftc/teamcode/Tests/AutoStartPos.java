package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@Autonomous(name="AutoStartPos", group="Util")

public class AutoStartPos extends LinearOpMode {

    private Arm arm=null;
    private Intake intake=null;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        while (!arm.getSlideLimitState()){
            arm.homeSlides();
        }
    }

}
