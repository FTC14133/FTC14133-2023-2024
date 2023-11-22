package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@TeleOp(name="LimitSwitch", group="Iterative Opmode") // Labels program in Driver station Selection

public class LimitSwitch extends OpMode {
    DigitalChannel slideLimit;

    public void init() {

        slideLimit = hardwareMap.get(DigitalChannel.class, "slideLS");
        slideLimit.setMode(DigitalChannel.Mode.INPUT);

    }



    public void loop() {
        telemetry.addData("Status", "Looping");
        telemetry.addData("ls", slideLimit.getState());
        telemetry.update();

    }

    public void stop(){

    }
}
