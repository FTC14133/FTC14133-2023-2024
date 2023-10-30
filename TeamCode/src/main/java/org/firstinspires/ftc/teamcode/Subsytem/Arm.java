package org.firstinspires.ftc.teamcode.Subsytem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    DcMotorEx slideM;
    DcMotorEx armLM, armRM;

    AnalogInput armPNP;

    private PIDController armController; // todo: make ff
    public static double armP = 0, armI = 0, armD = 0; // todo: tune pid

    private PIDController slideController; // todo: make ff
    public static double slideP = 0, slideI = 0, slideD = 0; // todo: tune pid

    boolean toggleArm = true;
    int armSlidePos = -1;

    int armMax = 3;
    int armMin = -1;

    double armTargetPos = 0;
    double slideTargetPos = 0;

    final double degpervoltage = 270/3.3;

    public Arm(HardwareMap hardwareMap){
        slideM = hardwareMap.get(DcMotorEx.class, "slideM");
        slideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLM = hardwareMap.get(DcMotorEx.class, "PivotArmLM");
        armRM = hardwareMap.get(DcMotorEx.class, "PivotArmRM");

        armPNP = hardwareMap.get(AnalogInput.class, "intakePNP");

        armController.setPID(armP, armI, armD);
        slideController.setPID(slideP, slideI, slideD);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry){
        if (toggleArm && (gamepad2.dpad_up || gamepad2.dpad_down)) {  // Only execute once per Button push
            toggleArm = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            if (gamepad2.dpad_down) {  // If the d-pad up button is pressed
                armSlidePos = armSlidePos + 1; //Increase Arm position
                if (armSlidePos > armMax) { //If arm position is above 3
                    armSlidePos = armMax; //Cap it at 3
                }
            } else if (gamepad2.dpad_up) { // If d-pad down button is pressed
                armSlidePos = armSlidePos - 1; //Decrease arm position
                if (armSlidePos < armMin) { //If arm position is below -3
                    armSlidePos = armMin; //cap it at -3
                }
            }

        }
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down) { //if neither button is being pressed
            toggleArm = true; // Button has been released, so this allows a re-press to activate the code above.
        }

        GoToPosition(armSlidePos);
    }

    public void GoToPosition(int position){

        armSlidePos = position; // to update in auto, redundant in teleop

        switch (armSlidePos){
            case -1:
                armTargetPos = 0; // todo: get good angle
                slideTargetPos = 0;
                break;
            case 0:
                armTargetPos = 1; // todo: get good angle
                slideTargetPos = 0;
                break;
            case 1:
                armTargetPos = 2; // todo: get good angle
                slideTargetPos = 0;
                break;
            case 2:
                armTargetPos = 3; // todo: get good angle
                slideTargetPos = 0;
                break;
            case 3:
                armTargetPos = 4; // todo: get good angle
                slideTargetPos = 0;
                break;
            default:
                throw new IllegalStateException("Unexpected position value: " + position); // todo: remove in comp
        }

        double armPower = armController.calculate(getArmAngle(), armTargetPos);
        double slidePower = slideController.calculate(getSlideLenght(), slideTargetPos);

        armLM.setPower(armPower);
        armRM.setPower(armPower);

        slideM.setPower(slidePower);

    }

    public double getArmAngle(){
        double PNPVoltage = armPNP.getVoltage();
        return degpervoltage*PNPVoltage;
    }

    public double getSlideLenght(){
        return slideM.getCurrentPosition();
    }


}
