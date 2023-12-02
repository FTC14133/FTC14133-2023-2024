package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "autoFTC14133")
public class Autonomous extends LinearOpMode {

    private Arm arm=null;
    private Intake intake=null;

    enum State {
        SPIKE,
        BACKDROP,
        ALIGN,
        PLACE,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;

    int armSlidePos = 1;
    double intakeState = 0;

    ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

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
                        break;
                    case "close":
                        startX = 12;
                        startY = -62;
                }
            case "blue":
                switch (side){
                    case "far":
                        startX = -35;
                        startY = 80;
                        break;
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
                .build();

        TrajectorySequence RfarLR = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToConstantHeading(new Vector2d(-36, -11))
                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -36), Math.toRadians(180))
                .build();



        Trajectory RbackL = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(40, -30))
                .build();

        Trajectory RbackC = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(40, -36))
                .build();

        Trajectory RbackR = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(40, -42))
                .build();


        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(61, -58), 0)
                .build();



        currentState = State.SPIKE;
        switch (spike){
            case "left":
                drive.followTrajectoryAsync(RspikeL);
                break;
            case "right":
                drive.followTrajectoryAsync(RspikeR);
                break;
            case "center":
                drive.followTrajectoryAsync(RspikeC);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case SPIKE:
                    if (!drive.isBusy()){
                        currentState = State.BACKDROP;

                        armSlidePos = 1;

                        switch (side){
                            case "close":
                                drive.followTrajectorySequenceAsync(RstraightTo);
                                break;
                            case "far":
                                switch (spike){
                                    case "left": case "right":
                                        drive.followTrajectorySequenceAsync(RfarLR);
                                        break;
                                    case "center":
                                        drive.followTrajectorySequenceAsync(RstraightTo);
                                        break;
                                }
                        }
                    }
                    break;

                case BACKDROP:
                    if (!drive.isBusy()){
                        currentState = State.ALIGN;

                        switch (spike) {
                            case "left":
                                drive.followTrajectoryAsync(RbackL);
                                break;
                            case "right":
                                drive.followTrajectoryAsync(RbackR);
                                break;
                            case "center":
                                drive.followTrajectoryAsync(RbackC);
                                break;
                        }
                    }
                    break;

                case ALIGN:
                    if (!drive.isBusy()){
                        currentState = State.PLACE;

                        intakeState = 1;
                        intakeTimer.reset();

                    }
                    break;

                case PLACE:
                    if (intakeTimer.seconds() == 2.0){
                        intakeState = 0;

                        currentState = State.PARK;
                        drive.followTrajectoryAsync(park);
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    break;
            }

            drive.update();

            arm.GoToPosition(armSlidePos);
            intake.objcatcher.runOutake(intakeState);
            intake.objpivot.updateIntakeAngle(arm, telemetry);


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
