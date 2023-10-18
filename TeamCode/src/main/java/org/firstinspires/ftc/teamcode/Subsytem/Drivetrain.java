
package org.firstinspires.ftc.teamcode.Subsytem;

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
    final double wheelD =75.0/25.4; // Diameter of the wheel (in inches)
    final double gearratio=(84.0/29.0)*(84.0/29.0)*(76.0/21.0); //Ratio of the entire drivetrain from the motor to the wheel
    final double rotationK = 0.45; //Scaling factor for rotation (Teleop) Todo: Determine a good scaling factor for this. Should also calculate for real based on wheel diameter and location on robot.

    final double countsperin = countsperrev*(gearratio)*(1/(Math.PI*wheelD));
    final double wheelBaseR = 13.859564; //Wheel base radius in inches (distance between center of wheels on (long) side)
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

        //Setting tolerance (stop if close to target)
        rf.setTargetPositionTolerance(tolerance);
        lf.setTargetPositionTolerance(tolerance);
        lb.setTargetPositionTolerance(tolerance);
        rb.setTargetPositionTolerance(tolerance);
    }

    public void GoToCoord(double targetx, double targety, double fieldTarAngle, double speed, Odometry odometry) {

        boolean moving = true;

        while (moving){

            List<Double> cur_coord = odometry.Return_Coords(true);
            double XPos = cur_coord.get(0);
            double YPos = cur_coord.get(1);

            double robotCurAngle = odometry.Return_Angle(false);
            double robotTarAngle = (360-robotCurAngle)+fieldTarAngle;
            double robotTarAngleRad = Math.toRadians(robotTarAngle);

            double distance = Math.sqrt(Math.pow(targetx-XPos, 2)+Math.pow(targety-YPos, 2));

            double forward = Math.cos(robotTarAngleRad)*distance;
            double strafe = Math.sin(robotTarAngleRad)*distance;
            double rotate = (robotTarAngleRad * inchesperdegrotation);

            double lfD = ((forward + strafe) + rotate);      //distance for leftfront
            double lbD = ((forward - strafe) + rotate);      //distance for leftback
            double rfD = ((forward - strafe) - rotate);      //distance for rightfront
            double rbD = ((forward + strafe) - rotate);      //distance for rightback

            // Converting inches to encoder counts
            int rfEncoderCounts = (int)(rfD * countsperin);
            int lfEncoderCounts = (int)(lfD * countsperin);
            int lbEncoderCounts = (int)(lbD * countsperin);
            int rbEncoderCounts = (int)(rbD * countsperin);

            double rotPower = (rotate/distance);

            if (rotPower == Double.NaN){
                rotPower = rotate;
            }

            double lfPower = (((forward + strafe)) + rotPower);
            double lbPower = (((forward - strafe)) + rotPower);
            double rfPower = (((forward - strafe)) - rotPower);
            double rbPower = (((forward + strafe)) - rotPower);

            double denominator = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));

            lfPower = (lfPower/denominator) * speed;
            lbPower = (lbPower/denominator) * speed;
            rfPower = (rfPower/denominator) * speed;
            rbPower = (rbPower/denominator) * speed;

            //Setting current pos to 0
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Setting where the motors need to go
            rf.setTargetPosition(rfEncoderCounts);
            lf.setTargetPosition(lfEncoderCounts);
            lb.setTargetPosition(lbEncoderCounts);
            rb.setTargetPosition(rbEncoderCounts);
            //How fast the motors need to go
            rf.setPower(rfPower);
            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);
            // Telling the motors to go to target pos
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if ((!(lf.isBusy())) && (!(rf.isBusy()))){
                moving = false;
            }
        }

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