package org.firstinspires.ftc.teamcode.Reference.LearnJava.System;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TestArmSubsystem {

    static DcMotorEx testArm;

    boolean toggleLift = true;
    int position = 1;

    final int encoderTicks = 28;

    public TestArmSubsystem(HardwareMap hardwareMap){
        // Motor Mapping
        testArm = hardwareMap.get(DcMotorEx.class, "testArm");
        testArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Update_Arm(int position){

        testArm.setPower(0.5);

        switch (position){
            case 1:
                testArm.setTargetPosition((encoderTicks/3) * (position-1));

            case 2:
                testArm.setTargetPosition((encoderTicks/3) * (position-1));

            case 3:
                testArm.setTargetPosition((encoderTicks/3) * (position-1));

            case 4:
                testArm.setTargetPosition((encoderTicks/3) * (position-1));

                break;
            default:
                throw new IllegalStateException("Unexpected value: " + position);
        }

        testArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Teleop(Gamepad gamepad2) {
        if (toggleLift && (gamepad2.dpad_up || gamepad2.dpad_down)) {
            toggleLift = false;
            if (gamepad2.dpad_down) {
                position = position + 1;
                if (position > 4) {
                    position = 4;
                }
            } else if (gamepad2.dpad_up) {
                position = position - 1;
                if (position < 1) {
                    position = 1;
                }
            }
        }

        Update_Arm(position);
    }
}
