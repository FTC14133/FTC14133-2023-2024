package org.firstinspires.ftc.teamcode.Subsystems;

// Intake

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    // Instantiate the motor variables
    static Servo intake;
    //CRServo intake_b;
    boolean toggle = true;
    boolean Possession = true; //Variable telling whether we have possession of a game piece or not
    DigitalChannel IntakeTouch; //The "beambreak" sensor is a type of IR sensor that detects if it vision is broken
    int intakeOpen = 1;
    boolean toggleIntake = true;

    public int beforeIntakeState = 0;

    public Intake(HardwareMap hardwareMap){                 // Motor Mapping
        intake = hardwareMap.get(Servo.class, "intake");      //Sets the names of the hardware on the hardware map
        //intake_b = hardwareMap.crservo.get("intake_b");
        // "DeviceName" must match the Config EXACTLY
        IntakeTouch = hardwareMap.get(DigitalChannel.class, "IntakeTouch");
        // Set motor direction based on which side of the robot the motors are on
    }
/*
    private void runIntake(double rotationSpeed){
        intake.setPower(rotationSpeed);
        //intake_b.setPower(-rotationSpeed);
    }

 */


    public void Update_intake(int intakeState){ //Standard intake function
        //beforeIntakeState = intakeState;
        if (intakeState == 1){
            intake.setPosition(1);
            Possession = false;
        }else{
            intake.setPosition(0);
            Possession = true;
        }
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) { //Code to be run in Op Mode void Loop at top level
        if (toggleIntake && gamepad2.right_bumper) {
            toggleIntake = false;  // Prevents this section of code from being called again until the Button is released and re-pressed

            intakeOpen = intakeOpen * -1;

        } else if (!gamepad2.right_bumper) {
            toggleIntake = true;
        }

        Update_intake(intakeOpen);
        telemetry.addData("Possession", Possession);
    }

    public void beambreak_print(Telemetry telemetry){ //Code to be run in Op Mode void Loop at top level
        telemetry.addData("possession", Possession);
        telemetry.addData("IntakeTouch", IntakeTouch.getState());

    }
    public boolean getPossession(){
        return Possession; //returns the variable from thr beambreak that identifies if we have a cone or not
    }

    public void Possession_Check(){
        if(IntakeTouch.getState()){
            Possession = true;
        }
        else{
            Possession = false;
        }
    }
}

