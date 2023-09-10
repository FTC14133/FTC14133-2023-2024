package org.firstinspires.ftc.teamcode.Reference.FTC_2021_2022_Reference.Subsystems_Reference;/*
package org.firstinspires.ftc.teamcode.Subsystems;

// Generic Lift

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Pivot_Arm {
    // Instantiate the lift motor variables
    private DcMotorEx lift;
    boolean Home = false;
    RevTouchSensor HomeSwitch; //Mapped to port n+1 (odd number ports)


    public int position = 0; // Integer position of the arm
    int tolerance = 0; // Encoder tolerance
    final double countsperrev = 28; // Counts per rev of the motor
    final double gearratio=3*4*5*4; //Ratio of the entire Pivot Arm from the motor to the arm
    final double countsperdegree=countsperrev*gearratio/360; //Converts counts per motor rev to counts per degree of arm rotation
    final int countsperdegreeint= 4; //(int)countsperdegree; //Converts to an integer value
    final double liftpower=0.75;
    float joystick_double;
    int joystick_int;

    boolean toggle = true;

    public Pivot_Arm(HardwareMap hardwareMap){                 // Motor Mapping
        lift = hardwareMap.get(DcMotorEx.class, "lift");//Sets the names of the hardware on the hardware map
        HomeSwitch = hardwareMap.get(RevTouchSensor.class, "HomeSwitch");
    // "DeviceName" must match the Config EXACTLY

        // Set motor direction based on which side of the robot the motors are on
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        position=3; //initial arm position
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry){ //Code to be run in Op Mode void Loop at top level

        joystick_double = gamepad2.right_stick_y*60;
        joystick_int = (int) joystick_double;

        if (Home==false){ //If arm is not homed
            HomeArm(); //Runs the homing sequence for the arm to reset it
        }
        else if (gamepad2.back){ //If the arm is homed, but the back button is pressed
            SetArmHome(false); //Set home variable to false (not-homed)
        }
        else { //When arm is homed and back button not pressed

            if (toggle && (gamepad2.dpad_up || gamepad2.dpad_down)) {  // Only execute once per Button push
                toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                    if (gamepad2.dpad_down) {  // If the d-pad up button is pressed
                    position = position + 1; //Increase Arm position
                    if (position > 3) { //If arm position is above 3
                        position = 3; //Cap it at 3
                    }
                } else if (gamepad2.dpad_up) { // If d-pad down button is pressed
                    position = position - 1; //Decrease arm position
                    if (position < -3) { //If arm position is below -3
                        position = -3; //cap it at -3
                    }
                }
            }
            else if (!gamepad2.dpad_up && !gamepad2.dpad_down) { //if neither button is being pressed
                toggle = true; // Button has been released, so this allows a re-press to activate the code above.
            }

            GotoPosition(position, joystick_int);
        }
        telemetry.addData("Home", Home);
        telemetry.addData("Arm Position", position);
        telemetry.addData("Target Position", lift.getTargetPosition());
        telemetry.addData("Encoder Position", lift.getCurrentPosition());
    }

    public void GotoPosition(int position, int joystick){
        lift.setPower(liftpower);        //Sets the power for the lift
        switch (position) {
            case 3: // Intake Front
                lift.setTargetPosition(0*countsperdegreeint+joystick);
                break;
            case 2: // Mid Level Front
                lift.setTargetPosition(-60*countsperdegreeint+joystick);
                break;

            case 1: //Upper Level Front
                lift.setTargetPosition(-100*countsperdegreeint+joystick);
                break;

            case 0: //Straight Up
                lift.setTargetPosition(-140*countsperdegreeint+joystick);
                break;

            case -1: //Upper Level Back
                lift.setTargetPosition(-185*countsperdegreeint+joystick);
                break;
            case -2: //Mid Level Back
                lift.setTargetPosition(-220*countsperdegreeint+joystick);
                break;
            case -3: // Intake Back
                lift.setTargetPosition(-290*countsperdegreeint+joystick);
                break;
            default:
                throw new IllegalStateException("Unexpected position value: " + position);
        }

    }

    public int GetArmPosition(){ // Returns the current position value of the arm
        return position;
    }

    public void SetArmHome(boolean home){ //Sets whether the arm is homed or not, used for homing during a match
        Home= home;
    }
    public boolean GetArmHome(){
        return Home;
    } //Gets whether the arm is homed or not

    public void HomeArm(){ //Method to home arm
        if (HomeSwitch.isPressed()==false){ //If the home switch is not pressed
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(.5); //run the motor towards the switch
        }
        else { //when the switch is pressed
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stop lift motor and set position to 0
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Change the run mode
            lift.setTargetPositionTolerance(tolerance); //Set the arm encoder tolerance
            Home=true; //Change value of Home to true
        }
    }

}

 */