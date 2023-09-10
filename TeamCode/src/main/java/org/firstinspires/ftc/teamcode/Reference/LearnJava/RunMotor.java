package org.firstinspires.ftc.teamcode.Reference.LearnJava;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="RunMotor", group="Iterative Opmode")

public class RunMotor extends OpMode{

    private DcMotorEx test_motor = null;

    @Override

    public void init(){
        test_motor = hardwareMap.get(DcMotorEx.class, "lf");
    }

    @Override
    public void loop() {
        test_motor.setPower(gamepad1.left_stick_x);
    }
}
