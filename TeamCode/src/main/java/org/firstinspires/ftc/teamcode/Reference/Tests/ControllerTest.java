package org.firstinspires.ftc.teamcode.Reference.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Controller_Test", group="Iterative Opmode")
@Disabled
public class ControllerTest extends OpMode  {

    private DcMotor test_motor = null;

    public void init() {
        //test_motor = hardwareMap.get(DcMotorEx.class, "turn_table");

    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {
        //Log.i("Hi", "HI");
        telemetry.addData("right_stick_y", String.valueOf(gamepad2.right_stick_y));
        telemetry.addData("right_stick_y", String.valueOf(gamepad2.right_stick_x));
        telemetry.addData("right_stick_y", String.valueOf(gamepad2.left_stick_y));
        telemetry.update();

        //test_motor.setPower(1);

    }

}

