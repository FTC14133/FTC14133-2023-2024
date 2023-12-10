package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsytem.Arm;
import org.firstinspires.ftc.teamcode.Subsytem.Intake;

@TeleOp(name="getpostele", group="Iterative Opmode") // Labels program in Driver station Selection

public class getpostele extends OpMode {

    private Arm arm=null;
    private Intake intake=null;


    public void init() {

        arm = new Arm(hardwareMap);

        intake = new Intake(hardwareMap);

    }

    public void start() {
        telemetry.addData("Status", "Start");
        telemetry.update();
    }


    public void loop() {
        telemetry.addData("Status", "Looping");

        telemetry.addData("getArmSlidePos", arm.getArmSlidePos());
        telemetry.addData("getSlideLenght", arm.getSlideLenght());
        telemetry.addData("getIntakeAngle", intake.objpivot.getIntakeAngle(telemetry));

    }

    public void stop(){

    }
}
