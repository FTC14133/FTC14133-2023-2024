package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="MoveSlides", group="Iterative Opmode") // Labels program in Driver station Selection
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class MoveSlides extends OpMode {

    static DcMotorEx slide;
    TouchSensor slideLimit;

    public void init() {

        slideLimit = hardwareMap.get(TouchSensor.class, "slideLS");

        slide = hardwareMap.get(DcMotorEx.class, "slideM");

        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void loop() {
        telemetry.addData("Status", "Looping");
        telemetry.addData("slide", slide.getCurrentPosition());
        telemetry.update();

        if (gamepad2.a){
            slide.setPower(-1);
            while (!slideLimit.isPressed()){

            }
            slide.setPower(0);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        slide.setPower(gamepad2.right_stick_y);

    }

    public void stop(){

    }
}
