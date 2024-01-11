package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Autonomous(name="AutoStartPos", group="Util")

public class AutoStartPos extends LinearOpMode {

    private Arm arm=null;
    private Intake intake=null;

    boolean homing = true;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();

        while (homing){
            arm.runSlides(-1);
            if (arm.getSlideLimitState()){
                arm.stopSlides();
                homing = false;
            }
        }
    }

}
