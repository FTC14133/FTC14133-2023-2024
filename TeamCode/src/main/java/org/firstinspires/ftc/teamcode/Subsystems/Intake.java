package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    // Instantiate the motor variables
    static DcMotorEx intake;
    static CRServo outtake;

    static CRServo intakePivotL;
    static CRServo intakePivotR;

    AnalogInput pivotIntakePNP;

    private PIDController intakePController; // todo: make ff
    public static double p = 0.1, i = 0, d = 0; // todo: tune intake pivot pid

    //boolean Possession = true; //Variable telling whether we have possession of a game piece or not

    final double suckerSpeed = 1; // todo: figure out good speed
    final double degpervoltage = 270/3.3;

    double intakeOffset = 0;

    double intakeTargetPos = 0; // todo: see what starting pos we want

    boolean toggleIntake = true;

    public Catcher objcatcher;
    public Pivot objpivot;

    double targetIntake = 0;

    boolean toggleManualIntake = true;
    int manualIntakeOn = 1;

    public Intake(HardwareMap hardwareMap){                 // Motor Mapping
        intake = hardwareMap.get(DcMotorEx.class, "intakeM");
        outtake = hardwareMap.get(CRServo.class, "outtakeS");

        intakePivotL = hardwareMap.get(CRServo.class, "intakePLS");
        intakePivotR = hardwareMap.get(CRServo.class, "intakePRS");

        intakePivotL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivotR.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotIntakePNP = hardwareMap.get(AnalogInput.class, "intakePNP");

        intakePController = new PIDController(p, i, d);

        objcatcher = new Catcher();
        objpivot = new Pivot();


    }

    public class Catcher {
        public void runIntake(double intakeState){ // 1 is intake, 0 is off, -1 is outtake
            intake.setPower(suckerSpeed *intakeState);
        }

        public void runOutake(double outtakeState){
            outtake.setPower(suckerSpeed *outtakeState);
        }

        public void Teleop(Gamepad gamepad1, Telemetry telemetry) { //Code to be run in Op Mode void Loop at top level

            int intakeState = 0;
            int outtakeState = 0;


            if (toggleIntake && gamepad1.y) {  // Only execute once per Button push
                toggleIntake = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            } else if (!gamepad1.y) { //if neither button is being pressed
                toggleIntake = true; // Button has been released, so this allows a re-press to activate the code above.
            }

            if (!toggleIntake){ //Intake
                intakeState = 1;
            }else if (gamepad1.b){ //Reverse Intake
                intakeState = -1;
            }

            if (gamepad1.x){ //Outtake
                outtakeState = 1;
            }else if (gamepad1.a){ //Reverse Outtake
                outtakeState = -1;
            }

            runIntake(intakeState);
            runOutake(outtakeState);
            //telemetry.addData("Possession", Possession);
        }

/*
    // todo: add back if using beam break
    public boolean getPossession(){
        return Possession; //returns the variable from thr beambreak that identifies if we have a cone or not
    }*/
    }

    public class Pivot {
        public void setIntakeOffset(double offset){
            intakeOffset = offset;
        }

        public void manualPivot(Gamepad gamepad2){
            intakePivotL.setPower(gamepad2.left_stick_y);
            intakePivotR.setPower(gamepad2.left_stick_y);
        }

        public double getIntakeAngle(Telemetry telemetry){
            double PNPVoltage = pivotIntakePNP.getVoltage();

            //telemetry.addData("intake voltage", (pivotIntakePNP.getVoltage()));
            double currentAngle = (degpervoltage*PNPVoltage);
/*            if (doneReset){
                pastAngle = currentAngle;
                doneReset = false;
            }
            if (Math.abs((pastAngle-intakeOffset) - currentAngle) > 35){
                intakeOffset += (pastAngle-intakeOffset)-currentAngle;
            }

            telemetry.addData("intake pure", (currentAngle));
            currentAngle += intakeOffset;
            telemetry.addData("intake offseted", (currentAngle));

            pastAngle = currentAngle;*/
            return currentAngle;
        }

        public void GoToAngle(double angle, Telemetry telemetry){
            intakeTargetPos = angle;

            double currentPos = getIntakeAngle(telemetry);
            double pid = intakePController.calculate(currentPos, intakeTargetPos);

            intakePivotL.setPower(pid);
            intakePivotR.setPower(pid);
        }

        public void updateIntakeAngle(Arm arm, Telemetry telemetry){

            if ((arm.getArmSlidePos() == 0) || (arm.getArmSlidePos() == 4) || (arm.getArmSlidePos() == 5)){
                //telemetry.addData("In Intake", "pickup");
                targetIntake = 113;
                GoToAngle(targetIntake, telemetry);
            }else if (arm.getArmSlidePos() == -1){
                GoToAngle(83, telemetry);
            }
            else
            {
                //telemetry.addData("In Intake", "backdrop align");
                targetIntake = 67+arm.getArmAngle();
                if (targetIntake < 82){
                    targetIntake = 82;
                }
                else if (targetIntake > 270){
                    targetIntake = 270;
                }
                GoToAngle(targetIntake,telemetry);
            }

            telemetry.addData("intake target", targetIntake);
        }

        public void toggleEdit(Gamepad gamepad2, Arm arm, Telemetry telemetry){
            if (gamepad2.left_stick_button && toggleManualIntake){
                toggleManualIntake = false;
                manualIntakeOn *= -1;
            }
            else if (!gamepad2.left_stick_button){
                toggleManualIntake = true;
            }

            if (manualIntakeOn == 1) {
                updateIntakeAngle(arm, telemetry);
            }else{
                manualPivot(gamepad2);
            }
        }

        public double getIntakeTargetPos(){
            return targetIntake;
        }

    }
}



