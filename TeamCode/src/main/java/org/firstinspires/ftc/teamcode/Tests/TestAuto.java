package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "TestAuto")
public class TestAuto extends LinearOpMode {

    //private Arm arm=null;
    //private Intake intake=null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //arm = new Arm(hardwareMap);
        //intake = new Intake(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-35.54, -62.77, Math.toRadians(180.00)));

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-35.54, -62.77, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-35.54, -33.92, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-35.54, -45.92))
                .splineToConstantHeading(new Vector2d(-55.85, -51.46), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-59.31, -38.08), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-59.54, -11.31, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(35.77, -11.31, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(39.92, -37.15, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(50.77, -36.46, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(45.46, -11.54, Math.toRadians(0.00)))
                .build();

        drive.followTrajectorySequence(untitled0);


    }

}