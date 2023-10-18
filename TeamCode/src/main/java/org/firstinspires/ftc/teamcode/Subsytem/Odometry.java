package org.firstinspires.ftc.teamcode.Subsytem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;

public class Odometry extends Thread{

    private static DcMotorEx paraEncoder;
    private static DcMotorEx perpEncoder;

    // The IMU sensor object
    IMU imu;
    // State used for updating telemetry

    private double currentAngle = 0;
    private double beforeAngle = 0;


    private double currentXPos = 0;
    private double beforeXPos = 0;

    private double currentYPos = 0;
    private double beforeYPos = 0;


    private double XPos = 0;
    private double YPos = 0;

    final double countsperrev = 28; // Counts per rev of the motor
    final double wheelD =75.0/25.4; // Diameter of the wheel (in inches)
    final double gearratio=(84.0/29.0)*(84.0/29.0)*(76.0/21.0); //Ratio of the entire drivetrain from the motor to the wheel

    final double countsperin = countsperrev*(gearratio)*(1/(Math.PI*wheelD));

    public Odometry(HardwareMap hardwareMap){

        paraEncoder = hardwareMap.get(DcMotorEx.class, "paraEncoder");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "perpEncoder");

        paraEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    public double Return_Angle(boolean pure){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = orientation.getYaw(AngleUnit.DEGREES);

        if (pure){
            return angle;
        }
        else {
            //Check if negative
            if (angle < 0) {
                return 180 + (180 - Math.abs(angle));
            }
            return angle;
        }
    }

    private void Record_Coords(){
        //Not Rotating and Moving
        if ((beforeAngle == currentAngle) && (!(currentXPos == beforeXPos) || !(currentYPos == beforeYPos))){
            double XMoved = currentXPos - beforeXPos;
            double YMoved = currentYPos - beforeYPos;

            XPos += XMoved;
            YPos += YMoved;
        }
    }

    public List<Double> Return_Coords(boolean inches){
        if (inches){
            return Arrays.asList(XPos*countsperin, YPos*countsperin);
        }else{
            return Arrays.asList(XPos, YPos);
        }
    }

    public void run(){
        beforeAngle = currentAngle;
        currentAngle = Return_Angle(false);


        beforeXPos = currentXPos;
        currentXPos = perpEncoder.getCurrentPosition();

        beforeYPos = currentYPos;
        currentYPos = paraEncoder.getCurrentPosition();

        Record_Coords();
    }
}
