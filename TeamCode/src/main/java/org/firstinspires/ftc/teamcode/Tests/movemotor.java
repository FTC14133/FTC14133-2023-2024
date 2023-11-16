package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

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

    final double degpervoltage = 270/3.3;

    public void init() {

        intakePivotL = hardwareMap.get(CRServo.class, "intakePLS");
        intakePivotR = hardwareMap.get(CRServo.class, "intakePRS");

        armPivotL = hardwareMap.get(DcMotorEx.class, "armLM");
        armPivotR = hardwareMap.get(DcMotorEx.class, "armRM");

        pivotIntakePNP = hardwareMap.get(AnalogInput.class, "intakePNP");

        intakeOuter = hardwareMap.get(CRServo.class, "outtakeS");
        intakeSucker = hardwareMap.get(DcMotorEx.class, "intakeM");

        slide = hardwareMap.get(DcMotorEx.class, "slideM");


        intakePivotL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivotR.setDirection(DcMotorSimple.Direction.FORWARD);

        armPivotL.setDirection(DcMotorSimple.Direction.FORWARD);
        armPivotR.setDirection(DcMotorSimple.Direction.FORWARD);
    }



    public void loop() {
        telemetry.addData("Status", "Looping");

        telemetry.addData("getArmSlidePos", (pivotIntakePNP.getVoltage() * degpervoltage));
        telemetry.update();

        intakePivotL.setPower(gamepad2.left_stick_y);
        intakePivotR.setPower(gamepad2.left_stick_y);

        armPivotL.setPower(gamepad1.left_stick_y);
        armPivotR.setPower(gamepad1.left_stick_y);

        telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);
        telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);

        slide.setPower(gamepad2.right_stick_y);

        if (gamepad2.x){
            intakeOuter.setPower(1);
        }

        if (gamepad2.y){
            intakeSucker.setPower(1);
        }

        intakeSucker.setPower(0);
        intakeOuter.setPower(0);

    }

    public void stop(){

    }
}
