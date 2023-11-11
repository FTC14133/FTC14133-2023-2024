
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Detection;
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

    private Detection Detection=null;

 public void init() {

     Detection = new Detection(hardwareMap);

 }

 public void start() {
     telemetry.addData("Status", "Start");
     telemetry.update();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     Detection.Teleop(telemetry, gamepad1);

     telemetry.update();

 }

 public void stop(){
     Detection.StopPortal();
 }
}
