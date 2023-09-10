package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.FinalOdo;

import java.lang.Math;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.*;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class OdometrySystem extends Thread{

    private Drivetrain drivetrain=null;

    private static DcMotorEx paraEncoder;
    private static DcMotorEx perpEncoder;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;

    public double currentAngle = 0;
    public double beforeAngle = 0;


    public double currentXPos = 0;
    public double beforeXPos = 0;

    public double currentYPos = 0;
    public double beforeYPos = 0;


    public double XPos = 0;
    public double YPos = 0;

    public OdometrySystem(HardwareMap hardwareMap){
        drivetrain = new Drivetrain(hardwareMap);

        paraEncoder = hardwareMap.get(DcMotorEx.class, "paraEncoder");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "perpEncoder");

        paraEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public double Return_Angle(){
        double angle = angles.firstAngle;

        //Check if negative
        if (angle < 0){
            return 180+(180-Math.abs(angle));
        }
        return angle;
    }

    public void Record_Coords(){
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

    public void GoToCoord(double gx, double gy, double speed, double direction, Telemetry telemetry){
        double realDirection = (360-currentAngle)+direction;
        double distance = Math.sqrt(Math.pow(gx-XPos, 2)+Math.pow(gy-YPos, 2));

        drivetrain.DrivetrainAutoMove(distance, speed, realDirection, telemetry);
    }

    public void run(){
        beforeAngle = currentAngle;
        currentAngle = Return_Angle();


        beforeXPos = currentXPos;
        currentXPos = perpEncoder.getTargetPosition();

        beforeYPos = currentYPos;
        currentYPos = paraEncoder.getTargetPosition();

        Record_Coords();
    }
}
