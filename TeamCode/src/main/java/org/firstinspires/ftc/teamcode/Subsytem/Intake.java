package org.firstinspires.ftc.teamcode.Subsytem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    // Instantiate the motor variables
    static CRServo intake;

    //boolean Possession = true; //Variable telling whether we have possession of a game piece or not

    int teleopState = 0;

    final double rotationSpeed = 1; // todo: figure out good speed

    public Intake(HardwareMap hardwareMap){                 // Motor Mapping
        intake = hardwareMap.get(CRServo.class, "intakeS");

    }

    private void runIntake(double intakeState){ // 1 is intake, 0 is off, -1 is outtake
        intake.setPower(rotationSpeed*intakeState);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) { //Code to be run in Op Mode void Loop at top level

        if (gamepad2.a){ //Intake
            teleopState = 1;
        }else if (gamepad2.b){ //Outtake
            teleopState = -1;
        }else{
            teleopState = 0;
        }

        runIntake(teleopState);
        telemetry.addData("teleopState", teleopState);
        //telemetry.addData("Possession", Possession);
    }

/*
    // todo: add back if using beam break
    public boolean getPossession(){
        return Possession; //returns the variable from thr beambreak that identifies if we have a cone or not
    }*/
}

