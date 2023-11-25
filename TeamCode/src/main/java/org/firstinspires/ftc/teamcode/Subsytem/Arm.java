package org.firstinspires.ftc.teamcode.Subsytem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    DcMotorEx slideM;
    DcMotorEx armL, armR;

    AnalogInput armPNP;

    TouchSensor slideLimit;

    private PIDFController armController;
    public static double armP = 0.1, armI = 0, armD = 0.00001, armFF = -0.001;

    boolean toggleArm = true;
    boolean toggleClimb = true;

    int climbOnOff = 1;

    int armSlidePos = 0;
    String clicklast = "b";

    int armMax = 4;
    int armMin = 0;

    double armTargetPos = 0;
    double slideTargetPos = 0;

    double slideStartPos = OpmodeStorage.slidePos;

    final double degpervoltage = 270/3.3;

    double slidePower = 1;

    public Arm(HardwareMap hardwareMap){
        slideM = hardwareMap.get(DcMotorEx.class, "slideM");
        slideM.setDirection(DcMotorSimple.Direction.REVERSE);
        slideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armL = hardwareMap.get(DcMotorEx.class, "armLM");
        armR = hardwareMap.get(DcMotorEx.class, "armRM");

        armL.setDirection(DcMotorSimple.Direction.FORWARD);
        armR.setDirection(DcMotorSimple.Direction.FORWARD);

        armPNP = hardwareMap.get(AnalogInput.class, "armPNP");

        slideLimit = hardwareMap.get(TouchSensor.class, "slideLS");

        armController = new PIDFController(armP, armI, armD, armFF);
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


        if (gamepad2.b){
            armSlidePos = 0;
            clicklast = "b";
        }else if (gamepad2.a){
            armSlidePos = 1;
            clicklast = "a";
        }else if (gamepad2.x){
            armSlidePos = 2;
            clicklast = "x";
        }


        if (gamepad2.back && toggleClimb){
            toggleClimb = false;
            climbOnOff *= -1;
            clicklast = "back";
        }else if (!gamepad2.back){
            toggleClimb = false;
        }

        if (clicklast.equals("back")) {
            if (climbOnOff == 1) {
                armSlidePos = 3;
            } else if (climbOnOff == -1) {
                armSlidePos = 4;
            }
        }


        GoToPosition(armSlidePos);
    }

    public void GoToPosition(int position){

        armSlidePos = position; // to update in auto, redundant in teleop

        switch (armSlidePos){
            case -1:
                armTargetPos = 111;
                slideTargetPos = 0;
                break;
            case 0: // Intake 96
                armTargetPos = 99;
                slideTargetPos = 33321;
                break;
            case 1: // Low Place
                armTargetPos = 69;
                slideTargetPos = 33321;
                break;
            case 2: // Medium Place
                armTargetPos = 63;
                slideTargetPos = 50592;
                break;
            case 3: // Climb High
                armTargetPos = 63; // todo: get good angle
                slideTargetPos = 0;
                break;
            case 4: // Climb Low
                armTargetPos = 63; // todo: get good angle
                slideTargetPos = 1;
                break;
            default:
                throw new IllegalStateException("Unexpected position value: " + position); // todo: remove in comp
        }

        slideTargetPos -= slideStartPos;

        slidePower = 1;
        if (slideLimit.isPressed()){;
            slideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideM.setTargetPosition(1000);
        }else{
            slideM.setTargetPosition((int) slideTargetPos);
        }

        slideM.setPower(slidePower);
        slideM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double armPower = armController.calculate(getArmAngle(), armTargetPos);
        armL.setPower(armPower);
        armR.setPower(armPower);

    }

    public void homeSlides(){
        slideM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideM.setPower(-1);
        while (!slideLimit.isPressed()){

        }
        slideM.setPower(0);
        slideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getArmAngle(){
        double PNPVoltage = armPNP.getVoltage();
        return degpervoltage*PNPVoltage;
    }

    public double getSlideLenght(){
        return slideM.getCurrentPosition();
    }

    public double getArmSlidePos(){
        return armSlidePos;
    }


}
