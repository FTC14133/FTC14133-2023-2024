
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

        drivetrain.DrivetrainAutoMove(10, 0.5, 45, 0, telemetry);

    }
}