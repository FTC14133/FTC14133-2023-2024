package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

public class  FTC_14133_2022 extends OpMode {

    private Arm arm=null;
    private Intake intake=null;


 public void init() {

     arm = new Arm(hardwareMap);

     intake = new Intake(hardwareMap);

 }

 public void start() {
     telemetry.addData("Status", "Start");
     telemetry.update();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     //arm.Teleop(gamepad2, telemetry);

     if (arm.getArmSlidePos() == -1){
         intake.objpivot.GoToAngle(0);
     }else{
         double targetIntake = 90-arm.getArmAngle();
         if (targetIntake < 0){
             targetIntake = 0;
         }
         intake.objpivot.GoToAngle(targetIntake);
     }

     telemetry.update();

 }

 public void stop(){

 }
}
