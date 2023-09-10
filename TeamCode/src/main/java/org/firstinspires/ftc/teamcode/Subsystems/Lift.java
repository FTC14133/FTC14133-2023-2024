
package org.firstinspires.ftc.teamcode.Subsystems;

// Generic Lift

import java.lang.Math;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.*;

//import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    // Instantiate the lift motor variables
    private DcMotorEx elevator;
    private DcMotorEx arm;
    boolean ElevatorHome = false;
    boolean ElevatorUpHome = false;
    boolean ArmHome = false;
    DigitalChannel HomeSwitchElevatorUp;
    DigitalChannel HomeSwitchElevatorDown;
    DigitalChannel HomeSwitchArmFront;
    DigitalChannel HomeSwitchArmBack;

    public int position = 0; // Integer position of the arm
    public int beforePosition = 8;
    int tolerance = 0; // Encoder tolerance
//<<<<<<< Updated upstream
    final double ArmCountsPerRev = 28; // Counts per rev of the motor
    final double ArmGearRatio = (32.0/10.0) * (76.0/21.0) * (68.0/13.0); //Ratio of the entire Pivot Arm from the motor to the arm
    final double ArmCountsPerDegree = ArmCountsPerRev * ArmGearRatio /360; //Converts counts per motor rev to counts per degree of arm rotation
//=======
    final double ElevatorCountsPerRev = 28; //Counts per revolution of the motor
    final double ElevatorGearRatio = (76.0/21.0)*(76.0/21.0)*(76.0/21.0); //Gear ratio of the motors
    final double ElevatorSpoolDiameter = 1; //The diameter of the wheels (We are converting mm to inch)
    final double ElevatorDTR = Math.PI* ElevatorSpoolDiameter / ElevatorGearRatio; //Distance traveled in one rotation
    final double ElevatorCountsPerInch = ElevatorCountsPerRev / ElevatorDTR; //Counts Per Inch

    double elevatorPower =1;
    double armPower=0.5;
    int joystick_int_left;
    int joystick_int_right;

    boolean toggleLift = true;
    boolean toggleFlip = true;

    int posNeg = 1;

    int topElevatorCount = 0;

    public static double armP = 14;
    //original P = 10
    public static double armI = 0.05;
    //original I = 0.05
    public static double armD = 0;
    //original D = 0
    public double armF = 0;
    //original F = 0

    PIDFCoefficients ArmPIDF = new PIDFCoefficients(armP, armI, armD, armF);

    public static double elevatorP = 14;
    //original P = 10
    public static double elevatorI = 0.05;
    //original I = 0.05
    public static double elevatorD = 0;
    //original D = 0
    public double elevatorF = 0;
    //original F = 0

    PIDFCoefficients elevatorPIDF = new PIDFCoefficients(elevatorP, elevatorI, elevatorD, elevatorF);

    public int simpBeforePosition = 0;
    public int simpPosition = 0;

    public boolean closeIntake = false;

    public Lift(HardwareMap hardwareMap){                 // Motor Mapping
        elevator = hardwareMap.get(DcMotorEx.class, "Elevator");//Sets the names of the hardware on the hardware map
        HomeSwitchElevatorUp = hardwareMap.get(DigitalChannel.class, "HSElevatorUp");
        HomeSwitchElevatorDown = hardwareMap.get(DigitalChannel.class, "HSElevatorDown");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");//Sets the names of the hardware on the hardware map
        HomeSwitchArmFront = hardwareMap.get(DigitalChannel.class, "HSArmFront");
        HomeSwitchArmBack = hardwareMap.get(DigitalChannel.class, "HSArmBack");
    // "DeviceName" must match the Config EXACTLY

        HomeSwitchElevatorUp.setMode(DigitalChannel.Mode.INPUT);
        HomeSwitchElevatorDown.setMode(DigitalChannel.Mode.INPUT);
        HomeSwitchArmFront.setMode(DigitalChannel.Mode.INPUT);
        HomeSwitchArmBack.setMode(DigitalChannel.Mode.INPUT);

        //arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, ArmPIDF);
        arm.setVelocityPIDFCoefficients(ArmPIDF.p, ArmPIDF.i, ArmPIDF.d, ArmPIDF.f);
        arm.setPositionPIDFCoefficients(ArmPIDF.p);

        // Set motor direction based on which side of the robot the motors are on
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        position=3; //initial arm position
        arm.setTargetPosition((int)(0 * ArmCountsPerDegree));
        elevator.setTargetPosition((int)(0 * ElevatorCountsPerInch));
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry){ //Code to be run in Op Mode void Loop at top level



        joystick_int_right = (int)(gamepad2.right_stick_y*60);
        joystick_int_left = (int)(gamepad2.left_stick_y*60);

        if ((!ElevatorHome) || (!ArmHome)){ //If arm is not homed
            Home(telemetry); //Runs the homing sequence for the arm to reset it
        }
        else if (gamepad2.back){ //If the arm is homed, but the back button is pressed
            SetArmHome(false); //Set home variable to false (not-homed)
            SetElevatorHome(false); //Set home variable to false (not-homed)
        }
        else if (toggleFlip && gamepad2.x){
            toggleFlip = false;
            position *= -1;
        }
        else if (!gamepad2.x){
            toggleFlip = true;
        }

        if (toggleLift && (gamepad2.dpad_up || gamepad2.dpad_down)) {  // Only execute once per Button push
            toggleLift = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            if (gamepad2.dpad_down) {  // If the d-pad up button is pressed
                position = position + 1; //Increase Arm position
                if (position > 8) { //If arm position is above 3
                    position = 8; //Cap it at 3
                }
            } else if (gamepad2.dpad_up) { // If d-pad down button is pressed
                position = position - 1; //Decrease arm position
                if (position < -8) { //If arm position is below -3
                    position = -8; //cap it at -3
                }
            }

        }
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down) { //if neither button is being pressed
            toggleLift = true; // Button has been released, so this allows a re-press to activate the code above.
        }

        if (gamepad2.left_bumper){
            posNeg = -1;
        }
// aditya was here
        if (gamepad2.b){
            position = 8 * posNeg;
        }else if (gamepad2.a){
            position = 3 * posNeg;
        }else if (gamepad2.x){
            position = 2 * posNeg;
        }else if (gamepad2.y){
            position = 1 * posNeg;
        }

        GotoPosition(position, joystick_int_left, joystick_int_right);

        posNeg = 1;

        telemetry.addData("ElevatorHome", ElevatorHome);
        telemetry.addData("ArmHome", ArmHome);
        telemetry.addData("Arm Position", position);
        telemetry.addData("Elev Target Position", elevator.getTargetPosition());
        telemetry.addData("Elev Encoder Position", elevator.getCurrentPosition());
        telemetry.addData("Arm Target Position", arm.getTargetPosition());
        telemetry.addData("Arm Encoder Position", arm.getCurrentPosition());
        telemetry.addData("armPIDF", "P = %.4f, I = %.4f, D = %.4f, F = %.4f", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p, arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i, arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d, arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f);

    }

    public void Limits(){
        if (!HomeSwitchArmFront.getState()){
            arm.setPower(Math.min(armPower, 0));
        }else if (!HomeSwitchArmBack.getState()){
            arm.setPower(Math.max(armPower, 0));
        }
        if (!HomeSwitchElevatorDown.getState()){
            elevator.setPower(Math.min(elevatorPower, 0));
        }else if (!HomeSwitchElevatorUp.getState()){
            elevator.setPower(Math.max(elevatorPower, 0));
        }
    }

    public void GotoPosition(int position, int Ljoystick, int Rjoystick){
        arm.setPower(armPower);
        elevator.setPower(elevatorPower);


/*        simpBeforePosition = beforePosition/Math.abs(beforePosition);
        simpPosition = position/Math.abs(position);

        if (simpBeforePosition != simpPosition){
            closeIntake = true;
        }*/


        //Limits();
        switch (position) {
            case 8: // Intake Front
                arm.setTargetPosition((int)(6 * ArmCountsPerDegree +Rjoystick)); //Todo: Determine all positions for the arm/lift
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 7:
                arm.setTargetPosition((int)(35+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 6:
                arm.setTargetPosition((int)(58+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 5:
                arm.setTargetPosition((int)(84+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 4:
                arm.setTargetPosition((int)(103+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 3: // Short Level Front
                arm.setTargetPosition((int)(255  +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case 2: // Mid Level Front
                arm.setTargetPosition((int)(74 * ArmCountsPerDegree +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((7 * ElevatorCountsPerInch) +Ljoystick));
                break;

            case 1: //Tall Level Front

                arm.setTargetPosition((int)(95 * ArmCountsPerDegree +Rjoystick));
                elevator.setTargetPosition((int)((10.5 * ElevatorCountsPerInch) +Ljoystick));
                break;

            case 0: //Straight Up
                arm.setTargetPosition((int)(111.6 * ArmCountsPerDegree +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((4.2 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -1: //Tall Level Back
                arm.setTargetPosition((int)(123 * ArmCountsPerDegree +Rjoystick));
                elevator.setTargetPosition((int)((10.5 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -2: //Mid Level Back
                arm.setTargetPosition((int)(145 * ArmCountsPerDegree +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((7 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -3: //Short Level Back
                arm.setTargetPosition((int)(785 +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -4:
                arm.setTargetPosition((int)(953+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -5:
                arm.setTargetPosition((int)(975+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -6:
                arm.setTargetPosition((int)(993+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -7:
                arm.setTargetPosition((int)(1021+Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            case -8: // Intake Back
                arm.setTargetPosition((int)(214 * ArmCountsPerDegree +Rjoystick));
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int)((0 * ElevatorCountsPerInch) +Ljoystick));
                break;
            default:
                throw new IllegalStateException("Unexpected position value: " + position);
        }

        //beforePosition = position;

    }

    //public boolean

    public int GetPosition(){ // Returns the current position value of the arm
        return position;
    }

    public void SetArmHome(boolean home){ //Sets whether the arm is homed or not, used for homing during a match
        ArmHome = home;
    }
    public boolean GetArmHome(){
        return ArmHome;
    } //Gets whether the arm is homed or not

    public void SetElevatorHome(boolean home){ //Sets whether the elevator is homed or not, used for homing during a match
        ElevatorHome = home;
    }

    public boolean GetElevatorHome(){
        return ElevatorHome;
    } //Gets whether the elevator is homed or not

    public void Home(Telemetry telemetry){
        Drivetrain.StopDrivetrain();
        while (!ElevatorHome){
            HomeElevator(telemetry);
        }
        while (!ArmHome){
            HomeArm(telemetry);
        }/*while (!ElevatorUpHome){
            HomeElevatorUp();
        }*/

    }

    public void HomeArm(Telemetry telemetry){ //Method to home arm
        if (HomeSwitchArmFront.getState()){ //If the home switch is not pressed
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-0.5); //run the motor towards the switch
            telemetry.addData("Homing Arm", "Homing");
            telemetry.update();
        }
        else { //when the switch is pressed
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stop lift motor and set position to 0
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Change the run mode
            arm.setTargetPositionTolerance(tolerance); //Set the arm encoder tolerance
            ArmHome =true; //Change value of Home to true
            telemetry.addData("Homing Arm", "Homed");
            telemetry.update();
        }
    }
    public void HomeElevator(Telemetry telemetry){ //Method to home arm

        if (HomeSwitchElevatorDown.getState()){ //If the home switch is not pressed
            elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator.setPower(-0.5); //run the motor towards the switch
            telemetry.addData("Homing Elev", "Homing");
            telemetry.update();
        }
        else { //when the switch is pressed
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stop lift motor and set position to 0
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Change the run mode
            elevator.setTargetPositionTolerance(tolerance); //Set the arm encoder tolerance
            ElevatorHome =true; //Change value of Home to true
            telemetry.addData("Homing Elev", "Homed");
            telemetry.update();
        }
    }
}
