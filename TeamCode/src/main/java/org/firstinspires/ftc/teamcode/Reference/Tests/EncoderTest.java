package org.firstinspires.ftc.teamcode.Reference.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ArmPositionTest", group="Iterative Opmode")
//@Disabled
public class EncoderTest extends OpMode  {
    private DcMotor arm = null;
    private DcMotor elevator = null;

    public void init() {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        elevator = hardwareMap.get(DcMotor.class, "Elevator");

        elevator.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void loop() {
        telemetry.addData("Elev Target Position", elevator.getCurrentPosition());
        telemetry.addData("Arm Target Position", arm.getCurrentPosition());
        telemetry.update();

    }

}

