
package org.firstinspires.ftc.teamcode.Reference.OdomotryTests.ThreadingTest;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ThreadingTeleop", group="Iterative Opmode")

public class ThreadingTeleop extends OpMode {

    public LoopTest thread;

    private static DcMotorEx rf;

    public void init() {

        thread = new LoopTest(hardwareMap, telemetry);
        thread.start();

        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rf.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public void loop(){

        rf.setPower(gamepad1.left_stick_x);

    }

    @Override
    public void stop() {
        thread.interrupt();
    }
}
