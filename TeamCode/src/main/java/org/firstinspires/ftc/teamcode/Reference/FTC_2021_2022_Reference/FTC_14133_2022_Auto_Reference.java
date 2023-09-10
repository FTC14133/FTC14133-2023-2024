package org.firstinspires.ftc.teamcode.Reference.FTC_2021_2022_Reference;/*
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot_Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Turn_Table;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")


//My favorite shape is a nonagon
//I like to ride dirt bikes RS


public class  FTC_14133_2022_Auto extends LinearOpMode {
    private Drivetrain drivetrain=null; // This activate the sub systems
    private Intake Intake=null;
    private Turn_Table Turn_Table=null;
    private Pivot_Arm Pivot_Arm =null;
    private Sensors Sensors=null;
    boolean GateFlag = false;
    boolean[] switches;
    boolean WT ; //This will decide if we are closer to the warehouse or turn table based on the switch on the robot
    boolean A ; //This will tell us that we are either on the red or blue alliance side
    double total_speed = 0.5; //This is the speed of most of the motors.


    public void waitForStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();
        //switches = Sensors.Update_Switches(); // Here we will see from the switches on the robot. Below is what they represent
        //WT = switches[0]; //This will decide if we are closer to the warehouse or turn table based on the switch on the robot
        //A = switches[1]; //This will tell us that we are either on the red or blue alliance side
        A = false;
        drivetrain = new Drivetrain(hardwareMap);
        Intake = new Intake(hardwareMap);
        Turn_Table = new Turn_Table(hardwareMap);
        Pivot_Arm = new Pivot_Arm(hardwareMap);
        Sensors = new Sensors(hardwareMap);
        telemetry.addData("Object Creation", "Done");
        telemetry.update();

    }

    public void runOpMode() {

        //     if ((Pivot_Arm != null) && (drivetrain != null) && (Intake !=null) && (Sensors != null) && (Turn_Table != null))
        //     {
        waitForStart();
        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();

        Pivot_Arm.SetArmHome(false);

        telemetry.addData("Object", "After SetArmHome");
        telemetry.update();
        Pivot_Arm.SetArmHome(false);
        while (Pivot_Arm.GetArmHome() == false) {
            Pivot_Arm.HomeArm(); //Runs the homing sequence for the arm to reset it
        }

        //Pivot_Arm.SetArmHome(true);

        telemetry.addData("Object", "Passed while loop");
        telemetry.update();

        Intake.Home_TSE();

        drivetrain.Rotate(-45, total_speed);
        Turn_Table.Auto(A, 8000);
        drivetrain.Rotate(-87, total_speed);
        Pivot_Arm.GotoPosition(1, 0); //Sets the arm to the position of top goal
        drivetrain.ForwardorBackwards(43, total_speed);
        Intake.Update_outtake(0.5, Pivot_Arm.position, gamepad2);
        drivetrain.Strafing(34, total_speed);
        drivetrain.Rotate(33, total_speed);
        drivetrain.ForwardorBackwards(60, total_speed);
        Pivot_Arm.GotoPosition(3, 0);



            //Pivot_Arm.GotoPosition(1); //Sets the arm to the position of top goal
            //drivetrain.Strafing(10, total_speed); //Line up towards shipping hub
            //drivetrain.ForwardorBackwards(26, total_speed); //Goes towards the shipping hub
            //Intake.Update_outtake(1, Pivot_Arm.position); //Places the freight on the correct level
            //drivetrain.ForwardorBackwards(-2.5, total_speed); //Moves backwards a bit
            //Pivot_Arm.GotoPosition(0); //57 and 26
            //Intake.Update_outtake(0, Pivot_Arm.position);
            //drivetrain.Rotate(-210.25, total_speed); //rotate to be in line of the turn table
            //drivetrain.Strafing(43.4, 0.2); //Goes to the turn table
            //Turn_Table.Auto(A, 4); //Runs the turn table
            //drivetrain.Rotate(15, total_speed); //Rotates to be in line with the storage hub
            //drivetrain.Strafing(-100, total_speed); //Goes to the storage hub


        if (A == false && WT == false && GateFlag == true) { //This code will check if the robot is on the BLUE side and on the Turntable side
            //Need Camera Code //Sees where the duck is

                //if camera == 1: //if the duck is on the first barcode
                //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the top of the shipping hub
                //if camera == 2: //if the duck is on the second barcode
                //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the middle of the shipping hub
                //if camera == 3: //if the duck is on the third barcode
                //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the bottom of the shipping hub


            drivetrain.Strafing(-12, total_speed); //Line up towards shipping hub
            drivetrain.ForwardorBackwards(35, total_speed); //Goes towards the shipping hub
            Intake.Update_outtake(1, Pivot_Arm.position, gamepad2); //Places the freight on the correct level
            drivetrain.ForwardorBackwards(-3, total_speed); //Moves backwards a bit
            drivetrain.Rotate(45, total_speed); //rotate to be in line of the turn table
            drivetrain.Strafing(56, total_speed); //Goes to the turn table
            Turn_Table.Auto(A, 1000); //Runs the turn table
            drivetrain.Rotate(-45, total_speed); //Rotates to be in line with the storage hub
            drivetrain.Strafing(-105, total_speed); //Goes to the storage hub


        } else if (A == false && WT == true && GateFlag == true) { //This is a different instance where if we are starting on the BLUE side and on the warehouse side
            //Need Camera Code //Sees where the duck is

            //if camera == 1: //if the duck is on the first barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the top of the shipping hub
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the top of the shipping hub
            //if camera == 2: //if the duck is on the second barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the middle of the shipping hub
            //if camera == 3: //if the duck is on the third barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the bottom of the shipping hub

            drivetrain.Strafing(12, total_speed); //We will go to the right to the team shipping hub
            drivetrain.ForwardorBackwards(35, total_speed); //goes forward to the team shipping hub
            Intake.Update_outtake(1, Pivot_Arm.position, gamepad2); //puts the freight on the shipping hub
            drivetrain.ForwardorBackwards(-35, total_speed); //goes away from the shipping hub
            drivetrain.Strafing(-62, total_speed); //goes up to the warehouse
        } else if (A == true && WT == false  && GateFlag == true) { //red and turntable side
            //Need Camera Code //Sees where the duck is

            //if camera == 1: //if the duck is on the first barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the top of the shipping hub
            //if camera == 2: //if the duck is on the second barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the middle of the shipping hub
            //if camera == 3: //if the duck is on the third barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the bottom of the shipping hub

            drivetrain.Strafing(12, total_speed); //Goes in line wit the shipping hub
            drivetrain.ForwardorBackwards(35, total_speed); //Goes to shipping hub
            Intake.Update_outtake(total_speed, Pivot_Arm.position, gamepad2); //Places freight
            drivetrain.ForwardorBackwards(-3, total_speed); //Goes backwards a bit
            drivetrain.Rotate(-45, total_speed); //Rotates to be in line with the turn table
            drivetrain.Strafing(56, total_speed); //Goes to the turntable
            Turn_Table.Auto(A, 1000); //Spins the turntable
            drivetrain.Rotate(45, total_speed); //Goes in line with the storage hub
            drivetrain.ForwardorBackwards(-105, total_speed); //Parks in the storage hub
        } else if (A == true && WT == true && GateFlag == true) { // red and warehouse side
            //Need Camera Code //Sees where the duck is

            //if camera == 1: //if the duck is on the first barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the top of the shipping hub
            //if camera == 2: //if the duck is on the second barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the middle of the shipping hub
            //if camera == 3: //if the duck is on the third barcode
            //    Pivot_Arm.Auto(3); //it will set the arm position to place the freight on the bottom of the shipping hub

            drivetrain.Strafing(-12, total_speed); // Lines up with shipping hub
            drivetrain.ForwardorBackwards(35, total_speed); //Goes towards the shipping hub
            Intake.Update_outtake(1, Pivot_Arm.position, gamepad2); //Runs the intake to release the fright
            drivetrain.ForwardorBackwards(-35, total_speed);//backs away from shipping hub
            drivetrain.Strafing(62, total_speed); //Parks in the storage hub
        }
    }
}
 */