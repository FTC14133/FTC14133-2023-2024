package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "autoFTC14133")
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);



        Trajectory spikeLF = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-47, -40))
                .build();
        TrajectorySequence spikeCF = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();
        Trajectory spikeRF = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-24, -36))
                .build();



        TrajectorySequence farLRB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToConstantHeading(new Vector2d(-36, -11))
                .lineToLinearHeading(new Pose2d(35, -59, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();

        TrajectorySequence farLRT = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToConstantHeading(new Vector2d(-36, -11))
                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();



        Trajectory finish = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();


        String alliance = "blue";
        String spike = "left";
        String side = "close";
        String lrMode = "top";

        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.x){
                alliance = "blue";
            }else if (gamepad1.b){
                alliance = "red";
            }

            if (gamepad1.dpad_left){
                spike = "left";
            }else if (gamepad1.dpad_right){
                spike = "right";
            }else if (gamepad1.dpad_up){
                spike = "center";
            }

            if (gamepad1.y){
                side = "far";
            }else if (gamepad1.a){
                side = "close";
            }

            if (gamepad1.right_bumper){
                lrMode = "top";
            }else if (gamepad1.left_bumper){
                lrMode = "bottom";
            }
        }

        if (!isStopRequested())

            if (alliance.equals("red")){
                if (side.equals("far")){
                    if (spike.equals("center")){
                        drive.followTrajectorySequence(spikeCF);
                    }else{
                        if (spike.equals("left")){
                            drive.followTrajectory(spikeLF);
                        }else{
                            drive.followTrajectory(spikeRF);
                        }


                        if (lrMode.equals("top")){
                            drive.followTrajectorySequence(farLRT);
                        }else{
                            drive.followTrajectorySequence(farLRB);
                        }
                    }
                }

            }

            //drive.followTrajectory(finish);
    }
}
