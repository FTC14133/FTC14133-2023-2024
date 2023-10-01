
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Detection;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
//import org.firstinspires.ftc.teamcode.Subsystems.Lights;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

//This is a test
//My favorite shape is a nonagon
//I like to ride dirt bikes RS

//TEST Commit Vihaan

//COMMENT YOUR CODE (VIHAAN)! We're adding a lot of automation, which is nice, but it is going to be REALLY difficult to troubleshoot.

public class  FTC_14133_2022 extends OpMode {
    private Drivetrain Drivetrain =null;
    //private Intake Intake=null;
    //private Lift Lift=null;
    //private Sensors Sensors=null;
    //private Lights Lights=null;
    private Detection Detection=null;
    boolean [] switches;
    boolean Alliance;
    boolean Warehouse_TurnTable;

 public void init() {
     Drivetrain = new Drivetrain(hardwareMap);
     //Intake = new Intake(hardwareMap);
     //Lift = new Lift(hardwareMap);
     //Sensors = new Sensors(hardwareMap);
     //Lights = new Lights(hardwareMap);
     Detection = new Detection(hardwareMap);

 }

 public void start() {
     telemetry.addData("Status", "Start");
     telemetry.update();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     //Drivetrain.Teleop(gamepad1, telemetry);
     Detection.Teleop(telemetry, gamepad1);

     telemetry.update();

 }

 public void stop(){
     Detection.StopPortal();
 }
}
