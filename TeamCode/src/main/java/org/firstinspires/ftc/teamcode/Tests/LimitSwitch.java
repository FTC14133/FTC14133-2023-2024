package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="LimitSwitch", group="Iterative Opmode") // Labels program in Driver station Selection
@com.qualcomm.robotcore.eventloop.opmode.Disabled
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
