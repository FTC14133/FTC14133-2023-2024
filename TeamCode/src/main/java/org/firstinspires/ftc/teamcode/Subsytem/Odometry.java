package org.firstinspires.ftc.teamcode.Subsytem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;

public class Odometry extends Thread{

    private Drivetrain drivetrain=null;

    private static DcMotorEx paraEncoder;
    private static DcMotorEx perpEncoder;

    // The IMU sensor object
    IMU imu;
    // State used for updating telemetry

    public double currentAngle = 0;
    public double beforeAngle = 0;


    public double currentXPos = 0;
    public double beforeXPos = 0;

    public double currentYPos = 0;
    public double beforeYPos = 0;


    public double XPos = 0;
    public double YPos = 0;

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

    public List<Object> Return_Coords(){
        return Arrays.asList(XPos, YPos);
    }

    public void GoToCoord(double targetx, double targety, double speed, double direction, Telemetry telemetry){
        double realDirection = (360-currentAngle)+direction;
        double distance = Math.sqrt(Math.pow(targetx-XPos, 2)+Math.pow(targety-YPos, 2));

        drivetrain.DrivetrainAutoMove(distance, speed, realDirection, telemetry);
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
