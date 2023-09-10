package org.firstinspires.ftc.teamcode.Reference.FTC_2021_2022_Reference.Subsystems_Reference;/*
package org.firstinspires.ftc.teamcode.Subsystems;

// Mecanum Drivetrain

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

public class Drivetrain  {
    // Instantiate the drivetrain motor variables
    private DcMotorEx lb; //Back left motor of drivetrain
    private DcMotorEx rb; //Back right motor of drivetrain
    private DcMotorEx lf; //Front left motor of drivetrain
    private DcMotorEx rf; //Front right motor of drivetrain
    int tolerance = 4; // Encoder tolerance
    final double countsperrev = 28; // Counts per rev of the motor
    final double wheelD =96/25.4; // Diameter of the wheel (in inches)
    final double gearratio=2*2.89*2.89; //Ratio of the entire drivetrain from the motor to the wheel
    final double countsperin=countsperrev*gearratio*(1/(Math.PI*wheelD));


    public Drivetrain(HardwareMap hardwareMap){                 // Motor Mapping
        lf = hardwareMap.get(DcMotorEx.class, "lf");      //Sets the names of the hardware on the hardware map
        rf = hardwareMap.get(DcMotorEx.class, "rf");      // "DeviceName" must match the Config EXACTLY
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Set motor direction based on which side of the robot the motors are on
        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void ForwardorBackwards(double distance, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setTargetPositionTolerance(tolerance);
        rf.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        //Driving forward/backwards
        double encodercounts = distance * countsperin;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(encodercountsint);
        lf.setPower(speed);        //Sets the power for the left front wheel
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the right front wheel
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the left back wheel
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the right back wheel
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || rf.isBusy()) {
            //run until motors arrive at position within tolerance
        }
    }

    public void Rotate(double turn, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setTargetPositionTolerance(tolerance);
        rf.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        //Driving left/right
        //NOT DONE
        double encodercounts = turn * 7.123; // test iteratively //ToDo: This math needs to be redone as well
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(-encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //noinspection StatementWithEmptyBody
        while (lf.isBusy() || rf.isBusy()) {
            //run until motors arrive at position within tolerance
        }
    }

    public void Strafing(double Strafe, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setTargetPositionTolerance(tolerance);
        rf.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        //Driving left/right
        //Positive is Strafing left negative is Strafing right
        double encodercounts = Strafe * countsperin * Math.sqrt(2);
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(-encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lf.isBusy() || rf.isBusy()) {
            //run until motors arrive at position within tolerance
        }


    }

    public void Teleop(Gamepad gamepad1, Telemetry telemetry){ //Code to be run in Teleop Mode void Loop at top level
        double leftPowerY = -gamepad1.left_stick_y;      //find the value of y axis on the left joystick;
        double leftPowerX = gamepad1.left_stick_x;      //find the value of x axis on the left joystick;
        double rightPowerX = gamepad1.right_stick_x*0.75;     //find the value of x axis on the right joystick;
        //Power of Mecanum wheels;
        double leftfrontpower = leftPowerY + leftPowerX + rightPowerX;     //Power level for leftfront
        double rightbackpower = leftPowerY + leftPowerX - rightPowerX;     //Power level for rightback
        double leftbackpower = leftPowerY - leftPowerX + rightPowerX;      //Power level for leftback
        double rightfrontpower = leftPowerY - leftPowerX - rightPowerX;    //Power level for rightfront

        if (gamepad1.right_bumper) {
            leftfrontpower *= 0.35;
            rightfrontpower *= 0.35;
            leftbackpower *= 0.35;
            rightbackpower *= 0.35;
        }

        //Get the max of the the absolute values of the power of the wheels.
        double NormScaling = Math.max(Math.max(Math.abs(leftfrontpower), Math.abs(rightfrontpower)), Math.max(Math.abs(leftbackpower), Math.abs(rightbackpower)));

        if (NormScaling > 1) {      //If the max of the the absolute values of the power of the wheels is greater than 1
            leftfrontpower /= NormScaling;       //Scales (divides) all of the powers of the wheels by the max of the the absolute values of the power of the wheels
            rightfrontpower /= NormScaling;
            leftbackpower /= NormScaling;
            rightbackpower /= NormScaling;
        } // Else just use the raw values

        //Set the power of the wheels
        lf.setPower(leftfrontpower);
        lb.setPower(leftbackpower);
        rf.setPower(rightfrontpower);
        rb.setPower(rightbackpower);

        telemetry.addData("LF Power", leftfrontpower);
        telemetry.addData("LB Power", leftbackpower);
        telemetry.addData("RF Power", rightfrontpower);
        telemetry.addData("RB Power", rightbackpower);

    }

    public double getWheelD() {
        return wheelD;
    }


}
*/