
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Detection;

import java.util.Objects;

@Autonomous(name="FTC_14133_2022_Auto", group="Auto")

public class  FTC_14133_2022_Auto extends LinearOpMode{
    private Drivetrain drivetrain=null; // This activates the sub systems
    private Intake Intake=null;
    private Lift Lift =null;
    private Sensors Sensors=null;
    private Detection Detection=null;
    private ColorSensorSubsystem Color = null;
    int routine = 7;
    int detected = 2;


    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        drivetrain = new Drivetrain(hardwareMap);
        Intake = new Intake(hardwareMap);
        Lift = new Lift(hardwareMap);
        Sensors = new Sensors(hardwareMap);
        Detection = new Detection(hardwareMap);
        Color = new ColorSensorSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        while (!opModeIsActive() && !isStopRequested()) {
            //Auto Routine Selector (Happens at the beginning of the match; do it quick!)
            telemetry.addData("Input Auto Routine (dpad_left = V5 = Terminal-Left, dpad_right = V6 = Terminal-Right, dpad_up = V7 = Park)", "");
            telemetry.addData("Alliance: ", "V"+(routine));
            if (gamepad1.dpad_left){
                routine = 5;
            }else if (gamepad1.dpad_right){
                routine = 6;
            }else if (gamepad1.dpad_up){
                routine = 7;
            }
            //Detection for April Tags
            detected = Detection.AprilTagDetection(telemetry);
            telemetry.update();
        }



        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();

        //Homing the lift
        Lift.SetArmHome(false);
        Lift.SetElevatorHome(false);
        Lift.Home(telemetry);

        telemetry.addData("Object", "After Home");
        telemetry.update();

        //Closing the intake at the beginning of auto
        Intake.Update_intake(0);
        sleep(1000);

        if (routine == 5){ //Left Side Auto
            drivetrain.DrivetrainAutoMove(13, 0.5, 270, telemetry); //Moves to be in terminal
            //drivetrain.DrivetrainAutoMove(2, 0.5, 360, telemetry);
            Intake.Update_intake(1); //Opens Intake
            sleep(3500);
            drivetrain.DrivetrainAutoMove(23.5, 0.5, 180, telemetry); // Moves to be in zone 1
            if (detected == 2){
                drivetrain.DrivetrainAutoMove(40, 0.5, 90, telemetry);// Moves to be in zone 2
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(90, 0.5, 90, telemetry);// Moves to be in zone 3
            }
        }else if (routine == 6) { //Right Side Auto
            drivetrain.DrivetrainAutoMove(44, 0.5, 90, telemetry); //Moves to be in terminal
            //drivetrain.DrivetrainAutoMove(2, 0.5, 270, telemetry);
            Intake.Update_intake(1); //Opens Intake
            sleep(3500);
            drivetrain.DrivetrainAutoMove(25, 0.5, 180, telemetry); // Moves to be in zone 3
            if (detected == 2) {
                drivetrain.DrivetrainAutoMove(20, 0.5, 270, telemetry); // Moves to be in zone 2
            } else if (detected == 1) {
                drivetrain.DrivetrainAutoMove(34, 0.5, 270, telemetry); // Moves to be in zone 1
            }

        }else if (routine == 7){
            drivetrain.DrivetrainAutoMove(27, 0.5, 180, telemetry);

            if (detected == 3){
                drivetrain.DrivetrainAutoMove(47, 0.5, 90, telemetry);
            }else if (detected == 1){
                drivetrain.DrivetrainAutoMove(27, 0.5, 270, telemetry);
            }
        }

/*        if (routine == -2){
            drivetrain.DrivetrainAutoMove(10, 0.5, 270, telemetry);

            drivetrain.DrivetrainStrafeColor(0.25, 270, 0, 0, telemetry);
            while (Objects.equals(Color.getColor(), "green")) {
                telemetry.addData("During wait", ".");
                telemetry.update();
            }

            StopDrivetrain();

            drivetrain.DrivetrainAutoMove(3, 0.5, 270, telemetry);
            drivetrain.DrivetrainAutoMove(36, 0.4, 180, telemetry);

            drivetrain.DrivetrainStrafeColor(0.25, -360, 0, 0, telemetry);
            while (Objects.equals(Color.getColor(), "green")) {
                telemetry.addData("During wait", ".");
                telemetry.update();
            }

            StopDrivetrain();

            Lift.GotoPosition(3, 0, 0);
            drivetrain.DrivetrainAutoMove(0.20, 50, telemetry);
            Intake.Update_intake(1);
            sleep(1500);
            drivetrain.DrivetrainAutoMove(0.20, 50, telemetry);
            drivetrain.DrivetrainAutoMove(4, 0.5, 90, telemetry);
            sleep(3000);
            Lift.GotoPosition(-4, 0, 0);
            sleep(3000);
            Intake.Update_intake(0);
            sleep(150);
            Lift.GotoPosition(2, 0, 0);
            sleep(10000);
            *//*
            drivetrain.DrivetrainAutoMove(16, 0.5, 360, telemetry);
            drivetrain.DrivetrainAutoMove(0.25, -50, telemetry);
            Intake.Update_intake(1);
            sleep(150);
            drivetrain.DrivetrainAutoMove(0.25, 50, telemetry);
            drivetrain.DrivetrainAutoMove(16, 0.5, 180, telemetry);
            Lift.GotoPosition(-3, 0, 0);
            sleep(2000);
            Intake.Update_intake(0);
            sleep(150);
            Lift.GotoPosition(3, 0, 0);
            sleep(2000);
            drivetrain.DrivetrainAutoMove(0.25, -45, telemetry);
            Intake.Update_intake(1);
            sleep(150);
            drivetrain.DrivetrainAutoMove(0.25, 45, telemetry);
            drivetrain.DrivetrainAutoMove(16, 0.5, 360, telemetry);
            drivetrain.DrivetrainAutoMove(22, 0.5, 270, telemetry);

            if (detected == 1){
                drivetrain.DrivetrainAutoMove(15, 0.75, 180, telemetry);
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(15, 0.5, 0, telemetry);
            }*//*



        }
        if (routine == -1){
            drivetrain.DrivetrainAutoMove(0.5, 50, telemetry);
            drivetrain.DrivetrainAutoMove(2.5, 0.5, 180, telemetry);

            Lift.GotoPosition(-3, 0, 0);
            sleep(1500);
            Intake.Update_intake(1);
            sleep(2000);
            drivetrain.DrivetrainAutoMove(0.5, -50, telemetry);
            drivetrain.DrivetrainAutoMove(44,  0.75, 180, telemetry);
            drivetrain.DrivetrainAutoMove(0.25, 90, telemetry);
            drivetrain.DrivetrainAutoMove(9, 0.5, 360, telemetry);
            Lift.GotoPosition(4, 0, 25);
            sleep(1000);
            drivetrain.DrivetrainAutoMove(1.5, 0.5, 360, telemetry);
            Intake.Update_intake(0);
            sleep(1000);
            Lift.GotoPosition(-2, 0, -70);
            drivetrain.DrivetrainAutoMove(30, 0.5, 180, telemetry);
            drivetrain.DrivetrainAutoMove(0.5, 77, telemetry);
            Intake.Update_intake(1);
        }
        if (routine == 0) { //This code will run if auto routine 0 is selected
            //drivetrain.DrivetrainAutoMove(12, 0.75, 0, telemetry);
            drivetrain.DrivetrainAutoMove(72, 0.5, 180, telemetry);
//hi
        }else if (routine == 1){ //
            Lift.GotoPosition(2, 0, 0);
            drivetrain.DrivetrainAutoMove(34, 0.75, 180, telemetry);
            drivetrain.DrivetrainAutoMove(0.75, -90, telemetry);
            Intake.Update_intake(1);
            sleep(3000);
            drivetrain.DrivetrainAutoMove(8, 0.75, 270, telemetry);
            if (detected == 1){
                drivetrain.DrivetrainAutoMove(15, 0.75, 180, telemetry);
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(15, 0.75, 0, telemetry);
            }
        }
        else if (routine == 2){
            telemetry.addData("detected", detected);
            telemetry.update();
            Lift.GotoPosition(2, 0, 0);
            drivetrain.DrivetrainAutoMove(36, 0.75, 180, telemetry);
            drivetrain.DrivetrainAutoMove(0.75, -90, telemetry);
            Intake.Update_intake(1);
            sleep(2000);
            drivetrain.DrivetrainAutoMove(23, 0.75, 90, telemetry);
            telemetry.addData("Current Detected Value", detected);
            telemetry.update();
            if (detected == 1){
                drivetrain.DrivetrainAutoMove(12, 0.5, 360, telemetry);
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(26, 0.5, 180, telemetry);
            }
        }
        else if (routine == 3){
            Lift.GotoPosition(2, 0, 0);
            drivetrain.DrivetrainAutoMove(30, 0.75, 0, telemetry);
            drivetrain.DrivetrainAutoMove(0.75, -90, telemetry);
            drivetrain.DrivetrainAutoMove(13, 0.75, 270, telemetry);
            if (detected == 1){
                drivetrain.DrivetrainAutoMove(15, 0.75, 0, telemetry);
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(15, 0.75, 180, telemetry);
            }
        }
        else if (routine == 4){ //This code will run if auto routine 4 is selected
            Lift.GotoPosition(1, 0, 0);
            drivetrain.DrivetrainAutoMove(30, 0.75, 180, telemetry);
            drivetrain.DrivetrainAutoMove(0.75, 90, telemetry);
            drivetrain.DrivetrainAutoMove(13, 0.75, 270, telemetry);
            if (detected == 1){
                drivetrain.DrivetrainAutoMove(15, 0.75, 180, telemetry);
            }else if (detected == 3){
                drivetrain.DrivetrainAutoMove(15, 0.75, 0, telemetry);
            }
        }
        else */
    }
}