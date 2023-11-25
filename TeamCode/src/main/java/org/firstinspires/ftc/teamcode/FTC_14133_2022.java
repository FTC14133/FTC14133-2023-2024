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

     //arm.homeSlides();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     intake.objcatcher.Teleop(gamepad2, telemetry);
     telemetry.addData("intake", intake.objpivot.getIntakeAngle());

     if (gamepad2.left_stick_button){
         arm.homeSlides();
     }

     arm.Teleop(gamepad2, telemetry);

     telemetry.addData("slidepos", arm.getSlideLenght());

     if (arm.getArmSlidePos() == 0){
         intake.objpivot.GoToAngle(145);
     }else{
         double targetIntake = 100+arm.getArmAngle();
         if (targetIntake < 86){
             targetIntake = 86;
         }else if (targetIntake > 270){
             targetIntake = 270;
         }
         telemetry.addData("targetIntake", targetIntake);
         intake.objpivot.GoToAngle(targetIntake);
     }

     telemetry.update();

 }

 public void stop(){

 }
}
