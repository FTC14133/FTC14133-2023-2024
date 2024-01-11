package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="movemotor", group="Iterative Opmode") // Labels program in Driver station Selection

public class movemotor extends OpMode {

    static DcMotorEx slide;

    static DcMotorEx intakeSucker;
    static CRServo intakeOuter;

    static CRServo intakePivotL;
    static CRServo intakePivotR;

    static DcMotorEx armPivotL;
    static DcMotorEx armPivotR;

    AnalogInput pivotIntakePNP;
    AnalogInput pivotArmPNP;

    final double degpervoltage = 270/3.3;

    public void init() {

        intakePivotL = hardwareMap.get(CRServo.class, "intakePLS");
        intakePivotR = hardwareMap.get(CRServo.class, "intakePRS");

        armPivotL = hardwareMap.get(DcMotorEx.class, "armLM");
        armPivotR = hardwareMap.get(DcMotorEx.class, "armRM");

        pivotIntakePNP = hardwareMap.get(AnalogInput.class, "intakePNP");
        pivotArmPNP = hardwareMap.get(AnalogInput.class, "armPNP");

        intakeOuter = hardwareMap.get(CRServo.class, "outtakeS");
        intakeSucker = hardwareMap.get(DcMotorEx.class, "intakeM");

        slide = hardwareMap.get(DcMotorEx.class, "slideM");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakePivotL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivotR.setDirection(DcMotorSimple.Direction.FORWARD);

        armPivotL.setDirection(DcMotorSimple.Direction.FORWARD);
        armPivotR.setDirection(DcMotorSimple.Direction.FORWARD);
    }



    public void loop() {
        telemetry.addData("Status", "Looping");


        telemetry.addData("intake deg", (pivotIntakePNP.getVoltage() * degpervoltage));
        telemetry.addData("intake voltage", (pivotIntakePNP.getVoltage()));
        telemetry.addData("arm", (pivotArmPNP.getVoltage() * degpervoltage));
        telemetry.addData("slide", slide.getCurrentPosition());
        telemetry.update();

        intakePivotL.setPower(-gamepad2.left_stick_y);
        intakePivotR.setPower(-gamepad2.left_stick_y);

        armPivotL.setPower(-gamepad1.left_stick_y);
        armPivotR.setPower(-gamepad1.left_stick_y);

        slide.setPower(-gamepad2.right_stick_y);

        if (gamepad2.x){
            intakeOuter.setPower(1);
        }else{
            intakeOuter.setPower(0);
        }

        if (gamepad2.y){
            intakeSucker.setPower(1);
        }else{
            intakeSucker.setPower(0);
        }
    }

    public void stop(){

    }
}
