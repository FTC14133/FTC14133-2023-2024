package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "autoFTC14133")
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        String[] selectedArray = autoSelector();

        String alliance = selectedArray[0];
        String spike = selectedArray[1];
        String side = selectedArray[2];

        double startX = 0;
        double startY = 0;

        switch (alliance){
            case "red":
                switch (side){
                    case "far":
                        startX = -35;
                        startY = -62;
                    case "close":
                        startX = 12;
                        startY = -62;
                }
            case "blue":
                switch (side){
                    case "far":
                        startX = -35;
                        startY = 80;
                    case "close":
                        startX = 12;
                        startY = 80;
                }
        }

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        Trajectory RspikeL = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-47, -40))
                .build();
        Trajectory RspikeC = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -36))
                .build();
        Trajectory RspikeR = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-24, -36))
                .build();



        TrajectorySequence RstraightTo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40, -36, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();

        TrajectorySequence RfarLR = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToConstantHeading(new Vector2d(-36, -11))
                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();





        Trajectory BspikeL = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-47, 40))
                .build();
        Trajectory BspikeC = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, 36))
                .build();
        Trajectory BspikeR = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-24, 36))
                .build();



        TrajectorySequence BstraightTo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40, 36, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(61, 58), 0)
                .build();

        TrajectorySequence BfarLR = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, 36))
                .lineToConstantHeading(new Vector2d(-36, 11))
                .lineToLinearHeading(new Pose2d(35, 11, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, 36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(61, 58), 0)
                .build();



        if (!isStopRequested()){
            switch (alliance){
                case "red":
                    switch (spike){
                        case "left":
                            drive.followTrajectory(RspikeL);
                            break;
                        case "right":
                            drive.followTrajectory(RspikeR);
                            break;
                        case "center":
                            drive.followTrajectory(RspikeC);
                            break;
                    }
                    switch (side){
                        case "close":
                            drive.followTrajectorySequence(RstraightTo);
                            break;
                        case "far":
                            switch (spike){
                                case "left": case "right":
                                    drive.followTrajectorySequence(RfarLR);
                                    break;
                                case "center":
                                    drive.followTrajectorySequence(RstraightTo);
                                    break;
                            }
                    }



                case "blue":
                    switch (spike){
                        case "left":
                            drive.followTrajectory(BspikeL);
                            break;
                        case "right":
                            drive.followTrajectory(BspikeR);
                            break;
                        case "center":
                            drive.followTrajectory(BspikeC);
                            break;
                    }
                    switch (side){
                        case "close":
                            drive.followTrajectorySequence(BstraightTo);
                            break;
                        case "far":
                            switch (spike) {

                                case "right": case "left":
                                    drive.followTrajectorySequence(BfarLR);
                                    break;

                                case "center":
                                    drive.followTrajectorySequence(BstraightTo);
                                    break;
                            }
                    }
            }
        }

    }

    public String[] autoSelector(){
        // Auto Selector
        String alliance = "blue";
        String spike = "left";
        String side = "close";
        String lrFMode = "top";

        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.x){
                alliance = "blue";
            }else if (gamepad1.b){
                alliance = "red";
            }
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", alliance.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.dpad_left){
                spike = "left";
            }else if (gamepad1.dpad_right){
                spike = "right";
            }else if (gamepad1.dpad_up){
                spike = "center";
            }
            telemetry.addData("Select Spike Mark (Gamepad1 D-PAD Left = Left Spike, Gamepad1 D-PAD Up = Center Spike, Gamepad1 D-PAD Right = Right Spike)", "");
            telemetry.addData("Current Spike Mark Selected : ", spike.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.y){
                side = "far";
            }else if (gamepad1.a){
                side = "close";
            }
            telemetry.addData("Select Side (Gamepad1 Y = Far, Gamepad1 A = Close)", "");
            telemetry.addData("Current Side Selected : ", side.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.right_bumper){
                lrFMode = "top";
            }else if (gamepad1.left_bumper){
                lrFMode = "bottom";
            }
            telemetry.addData("Select Left/Right Far Mode (Gamepad1 Right Bumper = Top, Gamepad1 Left Bumper = Bottom)", "");
            telemetry.addData("Current Left/Right Far Mode Selected : ", lrFMode.toUpperCase());
            telemetry.addData("", "");

            telemetry.update();
        }

        return new String[] {alliance, spike, side, lrFMode};

    }

}
