package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

public class  FTC_14133_2022 extends OpMode {

    private Arm arm=null;
    private Intake intake=null;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



 public void init() {

     arm = new Arm(hardwareMap);
     intake = new Intake(hardwareMap);

     drive = new Drive(hardwareMap);
     drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

     intake.objpivot.updateIntakeAngle(arm);




     Pose2d poseEstimate = drive.getPoseEstimate();

     Vector2d input = new Vector2d(
             -gamepad1.left_stick_y,
             -gamepad1.left_stick_x
     ).rotated(-poseEstimate.getHeading());

     drive.setWeightedDrivePower(
             new Pose2d(
                     input.getX(),
                     input.getY(),
                     -gamepad1.right_stick_x
             )
     );

     // Update everything. Odometry. Etc.
     drive.update();


     telemetry.update();

 }

 public void stop(){

 }
}
