package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TestOdo", group="Iterative Opmode")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
//@Disabled
public class TestOdo extends OpMode  {
    private DcMotor lOdo = null;
    private DcMotor rOdo = null;
    private DcMotor cOdo = null;

    public void init() {
        lOdo = hardwareMap.get(DcMotor.class, "armLM");
        rOdo = hardwareMap.get(DcMotor.class, "armRM");
        cOdo = hardwareMap.get(DcMotor.class, "intakeM");

        lOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        telemetry.addData("lOdo Position", lOdo.getCurrentPosition());
        telemetry.addData("rOdo Position", rOdo.getCurrentPosition());
        telemetry.addData("cOdo Position", cOdo.getCurrentPosition());
        telemetry.update();

    }

}