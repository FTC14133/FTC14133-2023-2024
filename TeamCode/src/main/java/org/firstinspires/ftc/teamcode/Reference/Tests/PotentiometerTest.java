package org.firstinspires.ftc.teamcode.Reference.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class PotentiometerTest extends LinearOpMode {
    // Define variables for our potentiometer and motor
    AnalogInput potentiometer;

    // Define variable for the current voltage
    double currentVoltage;

    @Override
    public void runOpMode() {
        // Get the potentiometer and motor from hardwareMap
        potentiometer = hardwareMap.get(AnalogInput.class, "PNP");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Update currentVoltage from the potentiometer
            currentVoltage = potentiometer.getVoltage();

            // Show the potentiometer’s voltage in telemetry
            telemetry.addData("Potentiometer voltage", currentVoltage);
            telemetry.update();
        }
    }
}