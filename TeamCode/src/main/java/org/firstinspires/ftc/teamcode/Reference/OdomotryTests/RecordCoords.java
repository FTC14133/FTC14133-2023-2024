package org.firstinspires.ftc.teamcode.Reference.OdomotryTests;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.lang.Math;

@TeleOp(name="RecordCoords", group="Iterative Opmode")
//@Disabled
public class RecordCoords extends OpMode  {
    private static DcMotorEx leftMotor;
    private static DcMotorEx rightMotor;

    private int previousEncoderValueLeftMotor = 0;
    private int previousEncoderValueRightMotor = 0;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    double total_x = 0;
    double total_y = 0;

    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "lf");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rf");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

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

    public void loop() {
        int currentEncoderValueLeftMotor = leftMotor.getCurrentPosition();
        int currentEncoderValueRightMotor = rightMotor.getCurrentPosition();

        boolean leftMotorMovingForward = currentEncoderValueLeftMotor > previousEncoderValueLeftMotor;
        boolean rightMotorMovingForward = currentEncoderValueRightMotor > previousEncoderValueRightMotor;

        telemetry.addData("Moving Forwards/Backwards:", (leftMotorMovingForward && rightMotorMovingForward));
        telemetry.addData("Robot Moved:", previousEncoderValueLeftMotor-currentEncoderValueLeftMotor);

        float current_heading = angles.firstAngle;

        if (leftMotorMovingForward && rightMotorMovingForward){

            int total_distance = previousEncoderValueLeftMotor-currentEncoderValueLeftMotor;

            double moved_x = (Math.cos(current_heading)*total_distance);
            double moved_y = (Math.sin(current_heading)*total_distance);

            total_x = total_x + moved_x;
            total_y = total_y + moved_y;
        }

        telemetry.addData("X Coord", total_x);
        telemetry.addData("Y Coord", total_y);
        telemetry.addData("Heading", angles.firstAngle);

        float real_angle = current_heading;

        if (current_heading < 0){ //Negative
            real_angle = 180 + (180+current_heading);
        }
        telemetry.addData("360 Heading", real_angle);

        telemetry.update();

    }
}
