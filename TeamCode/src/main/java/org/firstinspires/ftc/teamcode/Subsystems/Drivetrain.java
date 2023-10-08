
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
import java.util.List;
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
    final double gearratio=(84.0/29.0)*(84.0/29.0)*(76.0/21.0); //Ratio of the entire drivetrain from the motor to the wheel
    final double rotationK = 0.35; //Scaling factor for rotation (Teleop) Todo: Determine a good scaling factor for this. Should also calculate for real based on wheel diameter and location on robot.

    final double countsperin = countsperrev*(gearratio)*(1/(Math.PI*wheelD));
    final double wheelBaseR = 15.5/2.0; //Wheel base radius in inches
    final double inchesperdegrotation = 2 * Math.PI * wheelBaseR * (1.0/360.0);

    public Drivetrain(HardwareMap hardwareMap){                 // Motor Mapping
        lf = hardwareMap.get(DcMotorEx.class, "lf");      //Sets the names of the hardware on the hardware map
        rf = hardwareMap.get(DcMotorEx.class, "rf");      // "DeviceName" must match the Config EXACTLY
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Set motor direction based on which side of the robot the motors are on
        lb.setDirection(DcMotorEx.Direction.REVERSE);
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

    public void DrivetrainAutoMove(double distance, double speed, double direction, double rotation, Telemetry telemetry) {
        /*
         * Commands the robot to move a certain direction for a certain distance
         * Distance in inches, Speed in in/s, Direction in degrees (Front of robot is 0 deg, CCW is positive), Rotation in degrees (CCW is pos)
         */

        distance = distance*2;

        double angle = Math.toRadians(direction);

        double sin = Math.sin(angle - Math.PI/4);
        double cos = Math.cos(angle - Math.PI/4);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setTargetPositionTolerance(tolerance);
        lf.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);

        double denominator = Math.max(Math.abs(distance * cos), Math.max(Math.abs(distance * sin), 1.0));

        double lfPower = (distance * cos) / denominator;      //Speed for leftfront
        double lbPower = (distance * sin) / denominator;      //Speed for leftback
        double rfPower = (distance * sin) / denominator;      //Speed for rightfront
        double rbPower = (distance * cos) / denominator;      //Speed for rightback

        double lfD = (distance * cos);      //distance for leftfront
        double lbD = (distance * sin);      //distance for leftback
        double rfD = (distance * sin);      //distance for rightfront
        double rbD = (distance * cos);      //distance for rightback

        int rfEncoderCounts = (int)(lfD * countsperin);
        int lfEncoderCounts = (int)(rfD * countsperin);
        int lbEncoderCounts = (int)(rbD * countsperin);
        int rbEncoderCounts = (int)(lbD * countsperin);

        rf.setTargetPosition(rfEncoderCounts);
        lf.setTargetPosition(lfEncoderCounts);
        lb.setTargetPosition(lbEncoderCounts);
        rb.setTargetPosition(rbEncoderCounts);
        rf.setPower(rfPower);
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || rf.isBusy()/* || lb.isBusy() || rb.isBusy()*/) {
            telemetry.addData("rf.getCurrentPosition()", rf.getCurrentPosition());
            telemetry.addData("lf.getCurrentPosition()", lf.getCurrentPosition());
            telemetry.addData("lb.getCurrentPosition()", lb.getCurrentPosition());
            telemetry.addData("rb.getCurrentPosition()", rb.getCurrentPosition());

            telemetry.addData("lfPower", lfPower);
            telemetry.addData("lbPower", lbPower);
            telemetry.addData("rfPower", rfPower);
            telemetry.addData("rbPower", rbPower);

            telemetry.addData("rfencodercounts", rfEncoderCounts);
            telemetry.addData("lfencodercounts", lfEncoderCounts);
            telemetry.addData("lbencodercounts", lbEncoderCounts);
            telemetry.addData("rbEncoderCounts", rbEncoderCounts);
            telemetry.update();
        }

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

    public void GoToCoord(List<Double> curcoords, List<Double> tarcoords, double currentAngle, double speed, double direction, Telemetry telemetry){

        double XPos = curcoords.get(0);
        double YPos = curcoords.get(1);

        double targetx = tarcoords.get(0);
        double targety = tarcoords.get(1);

        double realDirection = (360-currentAngle)+direction;
        double distance = Math.sqrt(Math.pow(targetx-XPos, 2)+Math.pow(targety-YPos, 2));

        DrivetrainAutoMove(distance, speed, realDirection, telemetry);
    }

    public void Teleop(Gamepad gamepad1, Telemetry telemetry){ //Code to be run in Teleop Mode void Loop at top level
        double leftPowerY = -gamepad1.left_stick_y;      //find the value of y axis on the left joystick
        double leftPowerX = gamepad1.left_stick_x;      //find the value of x axis on the left joystick
        double rightPowerX = gamepad1.right_stick_x*rotationK;     //find the value of x axis on the right joystick

        double denominator = Math.max(Math.abs(leftPowerY) + Math.abs(leftPowerX) + Math.abs(rightPowerX), 1);

        double lfPower = (leftPowerY + leftPowerX + rightPowerX) / denominator;
        double lbPower = (leftPowerY - leftPowerX + rightPowerX) / denominator;
        double rfPower = (leftPowerY - leftPowerX - rightPowerX) / denominator;
        double rbPower = (leftPowerY + leftPowerX - rightPowerX) / denominator;


        if (gamepad1.right_bumper){
            lfPower = lfPower/2;
            lbPower = lbPower/2;
            rfPower = rfPower/2;
            rbPower = rbPower/2;
        }/*else { If we want slowed drivetrain
            lbpower = lbpower/1.5;
            rbpower = rbpower/1.5;
            lfpower = lfpower/1.5;
            rfpower = rfpower/1.5;
        }*/

        rf.setPower(rfPower);
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        telemetry.addData("RF Power", rfPower);
        telemetry.addData("LF Power", lfPower);
        telemetry.addData("LB Power", lbPower);
        telemetry.addData("RB Power", rbPower);

    }

}