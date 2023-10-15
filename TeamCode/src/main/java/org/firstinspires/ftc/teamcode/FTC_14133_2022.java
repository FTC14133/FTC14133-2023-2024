package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsytem.Odometry;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

public class  FTC_14133_2022 extends OpMode {

    private Drivetrain drivetrain=null;
    private Odometry odometry=null;


 public void init() {

     drivetrain = new Drivetrain(hardwareMap);
     odometry = new Odometry(hardwareMap);


 }

 public void start() {
     telemetry.addData("Status", "Start");
     telemetry.update();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     drivetrain.Teleop(gamepad1, telemetry);
     odometry.run();

     telemetry.addData("Robot Coords:", odometry.Return_Coords());
     telemetry.update();

 }

 public void stop(){

 }
}
