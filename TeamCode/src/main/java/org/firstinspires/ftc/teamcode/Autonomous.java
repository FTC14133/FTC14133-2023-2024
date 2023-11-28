package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "autoFTC14133")
public class Autonomous extends LinearOpMode {

    static final String SPIKE_CENTER = "center";
    static final String SPIKE_LEFT = "left";
    static final String SPIKE_RIGHT = "right";

    static final String SIDE_FAR = "far";
    static final String SIDE_CLOSE = "close";

    static final String ALLIANCE_RED = "red";
    static final String ALLIANCE_BLUE = "blue";


    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        String[] selectedArray = autoSelector();

        String alliance = selectedArray[0];
        String spike = selectedArray[1];
        String side = selectedArray[2];



        double startY = 0, startX = 0;

        if (side.equals(SIDE_FAR) && alliance.equals(ALLIANCE_RED)){
            startX = -35;
            startY = -64;
        }else if (side.equals(SIDE_CLOSE) && alliance.equals(ALLIANCE_RED)){
            startX = 12;
            startY = -64;
        }else if (side.equals(SIDE_FAR) && alliance.equals(ALLIANCE_BLUE)){
            startX = -35;
            startY = 64;
        }else if (side.equals(SIDE_CLOSE) && alliance.equals(ALLIANCE_BLUE)){
            startX = 12;
            startY = 64;
        }

        Pose2d startPose = new Pose2d(startX, startY, 0);
        drive.setPoseEstimate(startPose);

        int sideFlip = 1;
        if (side.equals(SIDE_CLOSE)){
            sideFlip = -1;
        }

        int allianceFlip = 1;
        if (alliance.equals(ALLIANCE_BLUE)){
            allianceFlip = -1;
        }



        Trajectory spikeL = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d((-47*sideFlip), (-42*allianceFlip)))
                .build();
        Trajectory spikeC = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d((-36*sideFlip), (-36*allianceFlip)))
                .build();
        Trajectory spikeR = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d((-24*sideFlip), (-42*allianceFlip)))
                .build();


        if (!isStopRequested()){
            switch (spike){
                case SPIKE_LEFT:
                    drive.followTrajectory(spikeL);
                    break;
                case SPIKE_RIGHT:
                    drive.followTrajectory(spikeR);
                    break;
                case SPIKE_CENTER:
                    drive.followTrajectory(spikeC);
                    break;
            }
            TrajectorySequence straightTo = updateStraightTo(drive, allianceFlip, sideFlip);
            TrajectorySequence farLR = updatefarLR(drive, allianceFlip);

            switchSide(drive, side, spike, straightTo, farLR);

            }
        }

    public TrajectorySequence updateStraightTo(SampleMecanumDrive drive, int allianceFlip, int sideFlip){

        telemetry.addData("poseEstimate", drive.getPoseEstimate());
        telemetry.update();

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d((-36*sideFlip), (-39*allianceFlip)))
                .lineToConstantHeading(new Vector2d(40, (-36*allianceFlip)))
                .splineToConstantHeading(new Vector2d(61, (-58*allianceFlip)), 0)
                .build();
    }

    public TrajectorySequence updatefarLR(SampleMecanumDrive drive, int allianceFlip){

        telemetry.addData("poseEstimate", drive.getPoseEstimate());
        telemetry.update();

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, (-48*allianceFlip)))
                .lineToConstantHeading(new Vector2d(-36, (-11*allianceFlip)))
                .lineToConstantHeading(new Vector2d(35, (-11*allianceFlip)))
                .splineToConstantHeading(new Vector2d(40, (-36*allianceFlip)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(61, (-58*allianceFlip)), 0)
                .build();
    }

    public void switchSide(SampleMecanumDrive drive, String side, String spike, TrajectorySequence straightTo, TrajectorySequence farLR){
        switch (side){
            case SIDE_CLOSE:
                drive.followTrajectorySequence(straightTo);
                break;
            case SIDE_FAR:
                switch (spike) {

                    case SPIKE_RIGHT: case SPIKE_LEFT:
                        drive.followTrajectorySequence(farLR);
                        break;

                    case SPIKE_CENTER:
                        drive.followTrajectorySequence(straightTo);
                        break;
                }
        }
    }

    public String[] autoSelector(){
        // Auto Selector
        String alliance = ALLIANCE_BLUE;
        String spike = SPIKE_LEFT;
        String side = SIDE_CLOSE;

        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.x){
                alliance = ALLIANCE_BLUE;
            }else if (gamepad1.b){
                alliance = ALLIANCE_RED;
            }
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", alliance.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.dpad_left){
                spike = SPIKE_LEFT;
            }else if (gamepad1.dpad_right){
                spike = SPIKE_RIGHT;
            }else if (gamepad1.dpad_up){
                spike = SPIKE_CENTER;
            }
            telemetry.addData("Select Spike Mark (Gamepad1 D-PAD Left = Left Spike, Gamepad1 D-PAD Up = Center Spike, Gamepad1 D-PAD Right = Right Spike)", "");
            telemetry.addData("Current Spike Mark Selected : ", spike.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.y){
                side = SIDE_FAR;
            }else if (gamepad1.a){
                side = SIDE_CLOSE;
            }
            telemetry.addData("Select Side (Gamepad1 Y = Far, Gamepad1 A = Close)", "");
            telemetry.addData("Current Side Selected : ", side.toUpperCase());
            telemetry.addData("", "");

            telemetry.update();
        }

        return new String[] {alliance, spike, side};

    }

}
