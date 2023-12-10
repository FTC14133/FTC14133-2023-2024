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

    private SampleMecanumDrive drive=null;

    double startPos = 129;

    boolean toggleManualIntake = true;
    int manualIntakeOn = 1;

    boolean toggleFieldCentric = true;
    int fieldCentricOn = 1;


 public void init() {

     arm = new Arm(hardwareMap);
     intake = new Intake(hardwareMap);

     drive = new SampleMecanumDrive(hardwareMap);
     drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


 }

 public void init_loop() {
     telemetry.addData("Status", "Init Loop");
     telemetry.update();

     arm.homeSlides();
 }

 public void start(){
     //intake.objpivot.setIntakeOffset(startPos-intake.objpivot.getIntakeAngle(telemetry));
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     intake.objcatcher.Teleop(gamepad1, telemetry);
     telemetry.addData("intake", intake.objpivot.getIntakeAngle(telemetry));

     if (gamepad2.right_stick_button){
         arm.homeSlides();
     }

     arm.Teleop(gamepad2, telemetry, intake);

     telemetry.addData("slidepos", arm.getSlideLenght());


     if (gamepad2.left_stick_button && toggleManualIntake){
         toggleManualIntake = false;
         manualIntakeOn *= -1;
     }
     else if (!gamepad2.left_stick_button){
         toggleManualIntake = true;
     }

     if (manualIntakeOn == 1) {
         intake.objpivot.updateIntakeAngle(arm, telemetry);
     }else{
         intake.objpivot.manualPivot(gamepad2);
     }




     Pose2d poseEstimate = drive.getPoseEstimate();

     Vector2d input = new Vector2d(
             -gamepad1.left_stick_y*0.5,
             -gamepad1.left_stick_x*0.5
     );//.rotated(-poseEstimate.getHeading());

     drive.setWeightedDrivePower(
             new Pose2d(
                     input.getX(),
                     input.getY(),
                     -gamepad1.right_stick_x*0.5
             )
     );

     // Update everything. Odometry. Etc.
     drive.update();


     telemetry.update();

 }

 public void stop(){

 }
}
