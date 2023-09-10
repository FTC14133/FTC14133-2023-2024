package org.firstinspires.ftc.teamcode.Reference.FTC_2021_2022_Reference.Subsystems_Reference;/*
package org.firstinspires.ftc.teamcode.Subsystems;

//This is an example subsystem. Reference this to set up new devices.

//demo on how the device works:
//https://www.youtube.com/watch?v=wMdkM2rr1a4&ab_channel=REVRobotics

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lights {
    RevBlinkinLedDriver blinkinLedDriver;

    public Lights(HardwareMap hardwareMap){ //Run this in Int to map the class items
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void Update_Lights(boolean possession,boolean Rotation,boolean A){
        if (possession==true){ //The possession is if we have freight in our robot
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }
        else if (Rotation==true){ //The Rotation is if the turn table motor is on
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
            }
        else if (A == true){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (A == false){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }
}

*/
