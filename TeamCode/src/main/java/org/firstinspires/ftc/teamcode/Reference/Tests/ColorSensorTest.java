package org.firstinspires.ftc.teamcode.Reference.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name="ColorSensorTest", group="Iterative Opmode")
public class ColorSensorTest extends OpMode {
    ColorSensor color_sensor;

    public void init() {

        color_sensor = hardwareMap.colorSensor.get("color");

    }
    public void loop(){
        telemetry.addData("Red", color_sensor.red());   // Red channel value
        telemetry.addData("Green", color_sensor.green()); // Green channel value
        telemetry.addData("Blue", color_sensor.blue());  // Blue channel value

        telemetry.addData("Alpha", color_sensor.alpha()); // Total luminosity
        telemetry.addData("ARGB", color_sensor.argb());  // Combined color value
    }
}
