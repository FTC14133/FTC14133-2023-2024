package org.firstinspires.ftc.teamcode.Reference.OdomotryTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="CheckIfMoving", group="Iterative Opmode")
//@Disabled
public class CheckIfMoving extends OpMode  {
    private static DcMotorEx lf; //Back left motor of drivetrain
    private static DcMotorEx rf; //Back right motor of drivetrain
    private static DcMotorEx lb; //Front left motor of drivetrain
    private static DcMotorEx rb; //Front right motor of drivetrain

    private int previousEncoderValueLeftMotor = 0;
    private int previousEncoderValueRightMotor = 0;

    public void init() {
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

    public void loop() {

        double lfPower = lf.getCurrentPosition();
        double rfPower = rf.getCurrentPosition();
        double lbPower = lb.getCurrentPosition();
        double rbPower = rb.getCurrentPosition();



        //Forward
        if (getDirection(lfPower) && !getDirection(rfPower) && getDirection(lbPower) && !getDirection(rbPower)){
            telemetry.addData("Moving:", "Forward");
        }
        //Backward
        else if (!getDirection(lfPower) && getDirection(rfPower) && !getDirection(lbPower) && getDirection(rbPower)){
            telemetry.addData("Moving:", "Backward");
        }
        //Left
        else if (!getDirection(lfPower) && !getDirection(rfPower) && getDirection(lbPower) && getDirection(rbPower)){
            telemetry.addData("Moving:", "Left");
        }
        //Right
        else if (getDirection(lfPower) && getDirection(rfPower) && !getDirection(lbPower) && !getDirection(rbPower)){
            telemetry.addData("Moving:", "Right");
        }

        telemetry.addData("lfPower", lf.getDirection());
        telemetry.addData("rfPower", lf.getCurrentPosition());
        telemetry.addData("lbPower", lf.getPower());
        telemetry.addData("rbPower", rbPower);
        telemetry.update();
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getDirection(double power){
        if (power < 0){
            return false;
        }
        return true;
    }
}
