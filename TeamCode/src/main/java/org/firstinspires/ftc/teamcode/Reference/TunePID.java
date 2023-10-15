package org.firstinspires.ftc.teamcode.Reference;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="TunePID", group="Iterative Opmode")
public class TunePID extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    private DcMotorEx motor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "testm");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int motorPos = motor.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);

        motor.setPower(pid);

        telemetry.addData("pos", motorPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
