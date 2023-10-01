package org.firstinspires.ftc.teamcode.Reference.LearnJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestTeleop", group="Auto")

public class TestAuto extends LinearOpMode {

    private TestArmSubsystem arm = null;
    private TestIntakeSubsystem intake = null;

    public void HardwareStart(){
        arm = new TestArmSubsystem(hardwareMap);
        intake = new TestIntakeSubsystem(hardwareMap);
    }

    @Override
    public void runOpMode() {

        intake.Update_intake(1);

        sleep(2500);

        arm.Update_Arm(3);

        sleep(2500);

        intake.Update_intake(0);

        sleep(2500);
    }
}
