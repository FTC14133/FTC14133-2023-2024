package org.firstinspires.ftc.teamcode.Reference.LearnJava.System;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestIntakeSubsystem {

    static Servo intake;
    int intakeOpen = 1;
    boolean toggleIntake = true;

    public TestIntakeSubsystem(HardwareMap hardwareMap){
        // Motor Mapping
        intake = hardwareMap.get(Servo.class, "intake");
    }

    public void Update_intake(int intakeState){
        //beforeIntakeState = intakeState;
        if (intakeState == 1){
            intake.setPosition(1);
        }else{
            intake.setPosition(0);
        }
    }

    public void Teleop(Gamepad gamepad2) {
        if (toggleIntake && gamepad2.right_bumper) {

            toggleIntake = false;
            intakeOpen = intakeOpen * -1;

        } else if (!gamepad2.right_bumper) {
            toggleIntake = true;
        }

        Update_intake(intakeOpen);
    }
}
