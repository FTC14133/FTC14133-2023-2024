package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Subsystems.TeamElementDetection.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "StateTest")
public class Autonomous extends LinearOpMode {

    static final String SPIKE_CENTER = "center";
    static final String SPIKE_LEFT = "left";
    static final String SPIKE_RIGHT = "right";

    static final String SIDE_FAR = "far";
    static final String SIDE_CLOSE = "close";

    static final String ALLIANCE_RED = "red";
    static final String ALLIANCE_BLUE = "blue";



    enum State {
        SPIKE,
        BACKDROP,
        SHAKE,
        ALIGN,
        PLACE,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;

    int armSlidePos = -1;
    int outtakeState = 0;
    int intakeState = 0;

    private Arm arm=null;
    private Intake intake=null;
    private TeamElementSubsystem teamElementDetection=null;
    private AprilTagDetection aprilTagDetection=null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        aprilTagDetection = new AprilTagDetection(hardwareMap);

        arm.resetSlideEncoder();

        String[] selectedArray = autoSelector();

        String alliance = selectedArray[0];
        String spike = selectedArray[1];
        String side = selectedArray[2];

        double startY = 0, startX = 0, startH = 0;

        if (side.equals(SIDE_FAR) && alliance.equals(ALLIANCE_RED)){
            startX = -36;
            startY = -63.75;
        }else if (side.equals(SIDE_CLOSE) && alliance.equals(ALLIANCE_RED)){
            startX = 12;
            startY = -63.75;
        }else if (side.equals(SIDE_FAR) && alliance.equals(ALLIANCE_BLUE)){
            startX = -36;
            startY = 63.75;
        }else if (side.equals(SIDE_CLOSE) && alliance.equals(ALLIANCE_BLUE)){
            startX = 12;
            startY = 63.75;
        }

        if (alliance.equals(ALLIANCE_RED)){
            startH = 180;
        }else{
            startH = 0;
        }

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startH));
        drive.setPoseEstimate(startPose);


        double sideOffset = 0;
        double specialSpikeOffset = 0;
        if (side.equals(SIDE_CLOSE)){
            sideOffset = 47;
            specialSpikeOffset = 23.5;
        }

        int allianceFlip = 1;
        if (alliance.equals(ALLIANCE_BLUE)){
            allianceFlip = -1;
        }



        TrajectorySequence spikeL = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46+sideOffset, -36.34*allianceFlip, Math.toRadians(startH)))
                .lineToLinearHeading(new Pose2d(-46+sideOffset, -41.34*allianceFlip, Math.toRadians(startH)))
                .build();
        TrajectorySequence spikeC = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36.34+sideOffset, -34.29*allianceFlip, Math.toRadians(startH)))
                .lineToLinearHeading(new Pose2d(-36.34+sideOffset, -39.29*allianceFlip, Math.toRadians(startH)))
                .build();
        TrajectorySequence spikeR = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-25.37+sideOffset, -36.34*allianceFlip, Math.toRadians(startH)))
                .build();

        if (alliance.equals(ALLIANCE_BLUE) && !(spike.equals(SPIKE_CENTER))){
            if (spike.equals(SPIKE_LEFT)){
                spikeL = spikeR;
            }else if (spike.equals(SPIKE_RIGHT)){
                spikeR = spikeL;
            }
        }

//26.5

        TrajectorySequence spikeSpecial = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36.34+sideOffset, -38.40*allianceFlip, Math.toRadians(startH)))
                .lineToLinearHeading(new Pose2d(-24.69+specialSpikeOffset, -34.74*allianceFlip, Math.toRadians(startH)))
                .lineToLinearHeading(new Pose2d(-24.69+specialSpikeOffset, -37.71*allianceFlip, Math.toRadians(startH)))
                .build();


        currentState = State.SPIKE;
        telemetry.addData("spike.equals(SPIKE_RIGHT) && side.equals(SIDE_CLOSE)", spike.equals(SPIKE_RIGHT) && side.equals(SIDE_CLOSE));
        telemetry.update();
        if ((alliance.equals(ALLIANCE_BLUE) &&
                (spike.equals(SPIKE_RIGHT) && side.equals(SIDE_CLOSE)
                || spike.equals(SPIKE_LEFT) && side.equals(SIDE_FAR)))
            ||
            alliance.equals(ALLIANCE_RED) &&
                (spike.equals(SPIKE_RIGHT) && side.equals(SIDE_FAR)
                || spike.equals(SPIKE_LEFT) && side.equals(SIDE_CLOSE))
        ){
            drive.followTrajectorySequenceAsync(spikeSpecial);
        }
        else {
            switch (spike) {
                case SPIKE_LEFT:
                    drive.followTrajectorySequenceAsync(spikeL);
                    break;
                case SPIKE_RIGHT:
                    drive.followTrajectorySequenceAsync(spikeR);
                    break;
                case SPIKE_CENTER:
                    drive.followTrajectorySequenceAsync(spikeC);
                    break;
            }
        }
        ElapsedTime intakeTimer = new ElapsedTime();
        ElapsedTime armTimer = new ElapsedTime();
        ElapsedTime shake1 = new ElapsedTime();
        ElapsedTime shake2 = new ElapsedTime();
        boolean resetArmTimer = true;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case SPIKE:
                    if (!drive.isBusy()){ // Done with spike movement
                        currentState = State.BACKDROP;

                        //armSlidePos = 1;

                        switch (side){
                            case SIDE_CLOSE:
                                armSlidePos = -3;
                                drive.followTrajectorySequenceAsync(updateStraightTo(drive, allianceFlip, sideOffset, side, alliance));
                                break;
                            case SIDE_FAR:
                                drive.followTrajectorySequenceAsync(updateFar(drive, allianceFlip));
                                break;
                        }
                    }
                    break;

                case BACKDROP:
                    if (!drive.isBusy()){
                        currentState = State.SHAKE;


                        shake1.reset();
                    }
                    break;

                case SHAKE:
                    if (shake1.milliseconds() < 250){
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        1,
                                        0,
                                        0
                                )
                        );
                        shake2.reset();
                    }else if (shake2.milliseconds() < 250){
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -1,
                                        0,
                                        0
                                )
                        );
                    }

                    else {
                        drive.setWeightedDrivePower(new Pose2d());

                        currentState = State.ALIGN;

                        double yBack = 0;
                        switch (alliance) {
                            case ALLIANCE_BLUE:
                                switch (spike) {
                                    case SPIKE_RIGHT:
                                        yBack = 30;
                                        break;
                                    case SPIKE_LEFT:
                                        yBack = 43;
                                        break;
                                    case SPIKE_CENTER:
                                        yBack = 34.5;
                                        break;
                                }
                                break;
                            case ALLIANCE_RED:
                                switch (spike) {
                                    case SPIKE_RIGHT:
                                        yBack = -42.5;
                                        break;
                                    case SPIKE_LEFT:
                                        yBack = -33.5;
                                        break;
                                    case SPIKE_CENTER:
                                        yBack = -36.5;
                                        break;
                                }
                                break;
                        }

                        telemetry.addData("yBack without offset", yBack);

                        telemetry.addData("yBack with offset", yBack);
                        telemetry.update();

                        drive.followTrajectoryAsync(updateBackAlign(drive, yBack));
                    }

                    break;

                case ALIGN:
                    if (!drive.isBusy() && resetArmTimer){
                        resetArmTimer = false;
                        armTimer.reset();
                    }
                    if (armTimer.seconds() >= 1.25 && !resetArmTimer){
                        currentState = State.PLACE;

                        outtakeState = 1;
                        intakeTimer.reset();

                    }
                    break;

                case PLACE:
                    if (intakeTimer.seconds() >= 5.0){
                        outtakeState = 0;

                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(updatePark(drive, allianceFlip));
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    //OpmodeStorage.slidePos = arm.getSlideLenght();
                    break;
            }

            drive.update();

            arm.GoToPosition(armSlidePos, intake, telemetry);
            intake.objpivot.updateIntakeAngle(arm, telemetry);
            intake.objcatcher.runOutake(outtakeState);
            intake.objcatcher.runIntake(intakeState);

            telemetry.addData("spike.equals(SPIKE_RIGHT)", spike.equals(SPIKE_RIGHT));
            telemetry.addData("spike.equals(SPIKE_RIGHT)", spike);
            telemetry.addData("side.equals(SIDE_CLOSE)", side.equals(SIDE_CLOSE));
            telemetry.update();


        }

    }

    public TrajectorySequence updateStraightTo(SampleMecanumDrive drive, int allianceFlip, double sideOffset, String side, String alliance){

        //telemetry.addData("poseEstimate", drive.getPoseEstimate());
        //telemetry.update();

        double turnAngle = 0;
        if (allianceFlip == 1){
            turnAngle = 180;
        }

        if (side.equals(SIDE_FAR)){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                    .addSpatialMarker(new Vector2d(0, -35*(allianceFlip)), () -> {
                        if (side.equals(SIDE_FAR)){
                            armSlidePos = -3;
                        }
                    })
                    .lineToConstantHeading(new Vector2d((42), (-38*allianceFlip)))
                    .lineToConstantHeading(new Vector2d(42, (-40*allianceFlip)))
                    .turn(Math.toRadians(turnAngle))
                    .build();
        }

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .addSpatialMarker(new Vector2d(0, -35*(allianceFlip)), () -> {
                    if (side.equals(SIDE_FAR)){
                        armSlidePos = -3;
                    }
                })
                .strafeLeft(7)
                .lineToConstantHeading(new Vector2d(42, (-40*allianceFlip)))
                .turn(Math.toRadians(turnAngle))
                .build();
    }

    public TrajectorySequence updateFar(SampleMecanumDrive drive, int allianceFlip){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .addSpatialMarker(new Vector2d(9, -11*(allianceFlip)), () -> {
                    armSlidePos = 1;
                })
                .addDisplacementMarker(() -> {
                    armSlidePos = 0;
                    intakeState = 1;
                })
                .lineToLinearHeading(new Pose2d(-57.5, -36.57, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-56, -10.97, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    intakeState = 0;
                    armSlidePos = -1;
                })
                .lineToLinearHeading(new Pose2d(38.86, -10.97, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(42.74, -36.57, Math.toRadians(0)))
                .build();
    }

    public Trajectory updateBackAlign(SampleMecanumDrive drive, double yValue){

        //telemetry.addData("poseEstimate", drive.getPoseEstimate());
        //telemetry.update();

        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(52, yValue))
                .build();
    }

    public TrajectorySequence updatePark(SampleMecanumDrive drive, int allianceFlip){

        //telemetry.addData("poseEstimate", drive.getPoseEstimate());
        //telemetry.update();

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(10)
                .lineToConstantHeading(new Vector2d(43.20, (-13.49*allianceFlip)))
                .build();
    }

    public String[] autoSelector(){
        // Auto Selector
        String alliance = ALLIANCE_BLUE;
        String spike = SPIKE_LEFT;
        String side = SIDE_CLOSE;

        while (!opModeIsActive() && !isStopRequested()){

            teamElementDetection.setAlliance(alliance);
            int element_zone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());


            if (gamepad1.x){
                alliance = ALLIANCE_BLUE;
            }else if (gamepad1.b){
                alliance = ALLIANCE_RED;
            }
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", alliance.toUpperCase());
            telemetry.addData("", "");

            if (element_zone == 1){
                spike = SPIKE_LEFT;
            }else if (element_zone == 3){
                spike = SPIKE_RIGHT;
            }else{
                spike = SPIKE_CENTER;
            }

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
