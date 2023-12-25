package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drone {

    CRServo droneS;

    ElapsedTime droneTimer = new ElapsedTime();

    boolean droneToggle = false;

    public Drone(HardwareMap hardwareMap){

        droneS = hardwareMap.get(CRServo.class, "droneS");
        droneS.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void Teleop(Gamepad gamepad2){
        if (gamepad2.back){
            droneToggle = true;
            droneTimer.reset();
        }

        if (droneToggle && (droneTimer.seconds() <= 3)){
            runDrone();
        }else{
            stopDrone();
        }
    }

    public void runDrone(){
        droneS.setPower(1);
    }

    public void stopDrone(){
        droneS.setPower(0);
    }
}
