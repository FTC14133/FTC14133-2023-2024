
package org.firstinspires.ftc.teamcode.Subsystems;

// Mecanum Drivetrain

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.lang.Math;
import java.util.Objects;

public class Drivetrain  {
    // Instantiate the drivetrain motor variables
    private static DcMotorEx lf; //Back left motor of drivetrain
    private static DcMotorEx rf; //Back right motor of drivetrain
    private static DcMotorEx lb; //Front left motor of drivetrain
    private static DcMotorEx rb; //Front right motor of drivetrain
    int tolerance = 4; // Encoder tolerance
    final double countsperrev = 28; // Counts per rev of the motor
    final double wheelD =96.0/25.4; // Diameter of the wheel (in inches)
    final double gearratio=(76.0/21.0)*(68.0/13.0); //Ratio of the entire drivetrain from the motor to the wheel
    final double countsperin=countsperrev*(gearratio)*(1/(Math.PI*wheelD));
    final double wheelBaseR = 15.5/2.0; //Wheel base radius in inches
    //final double wheelBaseR = wheelD/2.0; //Wheel base radius in inches
    final double rotationK = 0.35; //Scaling factor for rotation (Teleop) Todo: Determine a good scaling factor for this. Should also calculate for real based on wheel diameter and location on robot.
    final double maxSpeed = 6000 * countsperrev * (1.0/60.0); //Counts per S Todo: Determine the real max speed, likely through test
    final double inchesperdegrotation = 2 * Math.PI * wheelBaseR * (1.0/360.0);


    String colorValue = "green";

/*
    final double countsperrev = 28; // Counts per rev of the motor
    final double wheelD =96/25.4; // Diameter of the wheel (in inches)
    final double gearratio=2*2.89*2.89; //Ratio of the entire drivetrain from the motor to the wheel
    final double countsperin=countsperrev*gearratio*(1/(Math.PI*wheelD));
*/


    public Drivetrain(HardwareMap hardwareMap){                 // Motor Mapping
        lf = hardwareMap.get(DcMotorEx.class, "lf");      //Sets the names of the hardware on the hardware map
        rf = hardwareMap.get(DcMotorEx.class, "rf");      // "DeviceName" must match the Config EXACTLY
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Set motor direction based on which side of the robot the motors are on
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public static void StopDrivetrain(){
        lb.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        rf.setPower(0);
    }

    public void DrivetrainStrafeColor(double speed, double direction, double rotation, double distance, Telemetry telemetry){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setTargetPositionTolerance(tolerance);
        lf.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
        double angleR = Math.toRadians(direction)+(Math.PI/2); //Calculating angle of which the joystick is commanded to in radiant

        double rfspeed = -(Math.sin(angleR + (5 * Math.PI / 4)) * speed) - (rotation * rotationK);     //Speed for leftfront
        double lfspeed = -(Math.sin(angleR + (7 * Math.PI / 4)) * speed) - (rotation * rotationK);     //Speed for rightfront
        double lbspeed = -(Math.sin(angleR + (1 * Math.PI / 4)) * speed) - (rotation * rotationK);      //Speed for rightback
        double rbspeed = -(Math.sin(angleR + (3 * Math.PI / 4)) * speed) - (rotation * rotationK);    //Speed for leftback

        double maxNormalize = Math.max(Math.max(Math.abs(lfspeed), Math.abs(rfspeed)), Math.max(Math.abs(rbspeed), Math.abs(lbspeed))); //Finds the greatest power of the motors

        if ((Math.abs(lfspeed) > maxSpeed) || (Math.abs(rfspeed) > maxSpeed) || (Math.abs(rbspeed) > maxSpeed) || (Math.abs(lbspeed) > maxSpeed)){ //Normalize so no motor speed can be set above 1
            lfspeed = (lfspeed/maxNormalize) * maxSpeed;
            rfspeed = (rfspeed/maxNormalize) * maxSpeed;
            rbspeed = (rbspeed/maxNormalize) * maxSpeed;
            lbspeed = (lbspeed/maxNormalize) * maxSpeed;
        }

        double rfD = -((Math.sin(angleR + (5 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);     //direction for leftfront
        double lfD = -((Math.sin(angleR + (7 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);     //direction for rightfront
        double lbD = -((Math.sin(angleR + (1 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);      //direction for rightback
        double rbD = -((Math.sin(angleR + (3 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);    //direction for leftback

        int rfencodercounts = (int)(lfD * countsperin);
        int lfencodercounts = (int)(rfD * countsperin);
        int lbencodercounts = (int)(rbD * countsperin);
        int rbencodercounts = (int)(lbD * countsperin);

        if (distance > 0) {
            rf.setTargetPosition(rfencodercounts);
            lf.setTargetPosition(lfencodercounts);
            lb.setTargetPosition(lbencodercounts);
            rb.setTargetPosition(rbencodercounts);
        }
        rf.setPower(rfspeed);
        lf.setPower(lfspeed);
        lb.setPower(lbspeed);
        rb.setPower(rbspeed);

        if (distance > 0) {
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (distance >0) {
            while (lf.isBusy() || rf.isBusy()/* || lb.isBusy() || rb.isBusy()*/) {

                telemetry.addData("rf.getCurrentPosition()", rf.getCurrentPosition());
                telemetry.addData("lf.getCurrentPosition()", lf.getCurrentPosition());
                telemetry.addData("lb.getCurrentPosition()", lb.getCurrentPosition());
                telemetry.addData("rb.getCurrentPosition()", rb.getCurrentPosition());

                telemetry.addData("rfencodercounts", rfencodercounts);
                telemetry.addData("lfencodercounts", lfencodercounts);
                telemetry.addData("lbencodercounts", lbencodercounts);
                telemetry.addData("rbencodercounts", rbencodercounts);
                telemetry.update();
            }

            rf.setPower(0);
            lf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }


    }

    public void DrivetrainAutoMove(double distance, double speed, double direction, double rotation, Telemetry telemetry) {
        /*
         * Commands the robot to move a certain direction for a certain distance
         * Distance in inches, Speed in in/s, Direction in degrees (Front of robot is 0 deg, CCW is positive), Rotation in degrees (CCW is pos)
         */
        distance /=170;
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setTargetPositionTolerance(tolerance);
        lf.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
        double angleR = Math.toRadians(direction)+(Math.PI/2); //Calculating angle of which the joystick is commanded to in radiant

        double rfspeed = -(Math.sin(angleR + (5 * Math.PI / 4)) * speed) - (rotation * rotationK);     //Speed for leftfront
        double lfspeed = -(Math.sin(angleR + (7 * Math.PI / 4)) * speed) - (rotation * rotationK);     //Speed for rightfront
        double lbspeed = -(Math.sin(angleR + (1 * Math.PI / 4)) * speed) - (rotation * rotationK);      //Speed for rightback
        double rbspeed = -(Math.sin(angleR + (3 * Math.PI / 4)) * speed) - (rotation * rotationK);    //Speed for leftback

        double maxNormalize = Math.max(Math.max(Math.abs(lfspeed), Math.abs(rfspeed)), Math.max(Math.abs(rbspeed), Math.abs(lbspeed))); //Finds the greatest power of the motors

        if ((Math.abs(lfspeed) > maxSpeed) || (Math.abs(rfspeed) > maxSpeed) || (Math.abs(rbspeed) > maxSpeed) || (Math.abs(lbspeed) > maxSpeed)){ //Normalize so no motor speed can be set above 1
            rfspeed = (lfspeed/maxNormalize) * maxSpeed;
            lfspeed = (rfspeed/maxNormalize) * maxSpeed;
            lbspeed = (rbspeed/maxNormalize) * maxSpeed;
            rbspeed = (lbspeed/maxNormalize) * maxSpeed;
        }

        double rfD = -((Math.sin(angleR + (5 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);     //direction for leftfront
        double lfD = -((Math.sin(angleR + (7 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);     //direction for rightfront
        double lbD = -((Math.sin(angleR + (1 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);      //direction for rightback
        double rbD = -((Math.sin(angleR + (3 * Math.PI / 4)) * direction) * distance) - (rotation * inchesperdegrotation);    //direction for leftback

        int rfencodercounts = (int)(lfD * countsperin);
        int lfencodercounts = (int)(rfD * countsperin);
        int lbencodercounts = (int)(rbD * countsperin);
        int rbencodercounts = (int)(lbD * countsperin);

        rf.setTargetPosition(rfencodercounts);
        lf.setTargetPosition(lfencodercounts);
        lb.setTargetPosition(lbencodercounts);
        rb.setTargetPosition(rbencodercounts);
        rf.setPower(lfspeed);
        lf.setPower(rfspeed);
        lb.setPower(lbspeed);
        rb.setPower(rbspeed);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || rf.isBusy()/* || lb.isBusy() || rb.isBusy()*/) {
            telemetry.addData("rf.getCurrentPosition()", rf.getCurrentPosition());
            telemetry.addData("lf.getCurrentPosition()", lf.getCurrentPosition());
            telemetry.addData("lb.getCurrentPosition()", lb.getCurrentPosition());
            telemetry.addData("rb.getCurrentPosition()", rb.getCurrentPosition());

            telemetry.addData("rfencodercounts", rfencodercounts);
            telemetry.addData("lfencodercounts", lfencodercounts);
            telemetry.addData("lbencodercounts", lbencodercounts);
            telemetry.addData("rbencodercounts", rbencodercounts);
            telemetry.update();
        }

/*        while ((Math.abs(lf.getCurrentPosition()) < (Math.abs(lfencodercounts))) || (Math.abs(rf.getCurrentPosition()) < (Math.abs(rfencodercounts))) || (Math.abs(lb.getCurrentPosition()) < (Math.abs(lbencodercounts))) || (Math.abs(rb.getCurrentPosition()) < (Math.abs(rbencodercounts)))) {
            //run until motors arrive at position within tolerance
            telemetry.addData("rf.getCurrentPosition()", rf.getCurrentPosition());
            telemetry.addData("lf.getCurrentPosition()", lf.getCurrentPosition());
            telemetry.addData("lb.getCurrentPosition()", lb.getCurrentPosition());
            telemetry.addData("rb.getCurrentPosition()", rb.getCurrentPosition());

            telemetry.addData("rfencodercounts", rfencodercounts);
            telemetry.addData("lfencodercounts", lfencodercounts);
            telemetry.addData("lbencodercounts", lbencodercounts);
            telemetry.addData("rbencodercounts", rbencodercounts);
            telemetry.update();
        }*/

        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
    public void DrivetrainAutoMove(double distance, double speed, double direction, Telemetry telemetry){
        DrivetrainAutoMove(distance, speed, direction, 0, telemetry);
    }

    public void DrivetrainAutoMove(double speed, double rotation, Telemetry telemetry){
        DrivetrainAutoMove(0, speed, 0, rotation, telemetry);
    }

    public void Teleop(Gamepad gamepad1, Telemetry telemetry, boolean ArmHome, boolean ElevatorHome){ //Code to be run in Teleop Mode void Loop at top level
        double leftPowerY = -gamepad1.left_stick_y;      //find the value of y axis on the left joystick;
        double leftPowerX = gamepad1.left_stick_x;      //find the value of x axis on the left joystick;
        double rightPowerX = gamepad1.right_stick_x;     //find the value of x axis on the right joystick;


        double angleR = Math.atan2(leftPowerY, leftPowerX)+(Math.PI/2); //Calculating angle of which the joystick is commanded to in radians
        double angleD = Math.toDegrees(angleR); //Calculating angle of which the joystick is commanded to in degrees
        double speed = Math.sqrt((leftPowerY * leftPowerY) + (leftPowerX * leftPowerX)); //Calculating the magnitude of the joystick


        telemetry.addData("Angle: ", angleD);
        telemetry.addData("Speed: ", speed);

        double rfpower = -(Math.sin(angleR + (7 * Math.PI / 4)) * speed) + (rightPowerX * rotationK);    //Power level for rightfront
        double lfpower = -(Math.sin(angleR + (1 * Math.PI / 4)) * speed) + (rightPowerX * rotationK);    //Power level for leftfront
        double lbpower = -(Math.sin(angleR + (5 * Math.PI / 4)) * speed) + (rightPowerX * rotationK);    //Power level for leftback
        double rbpower = -(Math.sin(angleR + (3 * Math.PI / 4)) * speed) + (rightPowerX * rotationK);    //Power level for rightback


        double max = Math.max(Math.max(Math.abs(lfpower), Math.abs(rfpower)), Math.max(Math.abs(rbpower), Math.abs(lbpower))); //Finds the greatest power of the moters

        if ((Math.abs(lfpower) > 1) || (Math.abs(rfpower) > 1) || (Math.abs(rbpower) > 1) || (Math.abs(lbpower) > 1)){ //Normalize so no motor speed can be set above 1
            rfpower /= max;
            lfpower /= max;
            lbpower /= max;
            rbpower /= max;
        }

        if((!ArmHome) && (!ElevatorHome)){
            rfpower = 0;
            lfpower = 0;
            lbpower = 0;
            rbpower = 0;
        }
        if (gamepad1.right_bumper){
            lbpower = lbpower/2;
            rbpower = rbpower/2;
            lfpower = lfpower/2;
            rfpower = rfpower/2;
        }else {
            lbpower = lbpower/1.5;
            rbpower = rbpower/1.5;
            lfpower = lfpower/1.5;
            rfpower = rfpower/1.5;
        }

        rf.setPower(lbpower);
        lf.setPower(rbpower);
        lb.setPower(lfpower);
        rb.setPower(rfpower);

        telemetry.addData("RF Power", rfpower);
        telemetry.addData("LF Power", lfpower);
        telemetry.addData("LB Power", lbpower);
        telemetry.addData("RB Power", rbpower);

    }

    public double getWheelD() {
        return wheelD;
    }

    public void AutoStop(){
        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }


}