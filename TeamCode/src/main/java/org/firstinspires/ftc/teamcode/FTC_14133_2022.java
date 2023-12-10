package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Drone;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

public class  FTC_14133_2022 extends OpMode {

    private Arm arm=null;
    private Intake intake=null;
    private Drone drone=null;

    private SampleMecanumDrive drive=null;


 public void init() {

     arm = new Arm(hardwareMap);
     intake = new Intake(hardwareMap);
     drone = new Drone(hardwareMap);

     drive = new SampleMecanumDrive(hardwareMap);
     drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


 }

 public void init_loop() {
     telemetry.addData("Status", "Init Loop");
     telemetry.update();

     arm.homeSlides();
 }

 public void loop() {
     telemetry.addData("Status", "Looping");

     intake.objcatcher.Teleop(gamepad1, telemetry);
     intake.objpivot.toggleEdit(gamepad2, arm, telemetry);

     arm.Teleop(gamepad2, telemetry, intake);

     drone.Teleop(gamepad2);

     teleopDrive();

     telemetry.update();

 }

 public void teleopDrive(){
     Vector2d input = new Vector2d(
             -gamepad1.left_stick_y*0.75,
             -gamepad1.left_stick_x*0.75
     );

     drive.setWeightedDrivePower(
             new Pose2d(
                     input.getX(),
                     input.getY(),
                     -gamepad1.right_stick_x*0.5
             )
     );

     // Update everything. Odometry. Etc.
     drive.update();
 }
}
