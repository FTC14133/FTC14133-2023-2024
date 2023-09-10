package org.firstinspires.ftc.teamcode.Reference.Tests;

//This is an example subsystem. Reference this to set up new devices.

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Subsystem {

    DcMotorEx subsystemmotor = null; //Instantiate a motor
    Servo subsystemservo=null; //Instantiate a servo
    DigitalChannel subsystemdigitalsensor =null; //Instantiate a digital sensor
    AnalogSensor subsystemanalogsensor=null; //Instantiate an analog sensor
    boolean digitalvariable; //Instantiate a boolean variable
    double analogvariable; //Instantiate an analog variable
    int subsystemint; //Instantiate an integer variable


    public Subsystem(HardwareMap hardwareMap){ //Run this in Int to map the class items
    subsystemmotor = hardwareMap.get(DcMotorEx.class, "subsystemmotor"); //Maps the motor on the Robot Controller
    subsystemservo = hardwareMap.get(Servo.class, "subsystemservo"); //Maps the servo on the Robot Controller
    subsystemdigitalsensor = hardwareMap.get(DigitalChannel.class, "subsystemdigitalsensor"); //Maps the digital sensor on the Robot Controller
    subsystemanalogsensor = hardwareMap.get(AnalogSensor.class, "subsystemanalogsensor"); //Maps the analog sensor on the Robot Controller
    }

    public void updatesubsystem(double speed){ //Run this inside of the main program.
        subsystemmotor.setPower(speed); //Sets speed of the motor
        subsystemservo.setPosition(1); //Sets the position of the servo
        digitalvariable = subsystemdigitalsensor.getState(); //Gets the state of a digital sensor
        analogvariable = subsystemanalogsensor.readRawVoltage(); //Gets the voltage reading of an analog sensor

    }
}
