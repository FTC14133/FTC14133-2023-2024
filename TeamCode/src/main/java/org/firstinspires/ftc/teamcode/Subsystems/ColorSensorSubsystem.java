package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorSubsystem {
    ColorSensor color_sensor;

    String color = "green"; //Blue is True and Red is False

    int redVal = 0;
    int greenVal = 0;
    int blueVal = 0;


    public ColorSensorSubsystem(HardwareMap hardwareMap){                 // Motor Mapping
        color_sensor = hardwareMap.get(ColorSensor.class, "color");
        // Set motor direction based on which side of the robot the motors are on
    }

    public String getColor(){
        redVal = color_sensor.red();
        greenVal = color_sensor.green();
        blueVal = color_sensor.blue();

        if (redVal > 250){
            color = "red";
        }else if (blueVal > 350){
            color = "blue";
        }else{
            color = "green";
        }
        return color;
    }


}

