package org.firstinspires.ftc.teamcode.Subsytem;

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

    double  intakeTargetPos = 0; // todo: see what starting pos we want
    public Catcher objcatcher;
    public Pivot objpivot;

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

        public void Teleop(Gamepad gamepad2, Telemetry telemetry) { //Code to be run in Op Mode void Loop at top level

            int intakeState = 0;
            int outtakeState = 0;

            if (gamepad2.y){ //Intake
                intakeState = 1;
            }else if (gamepad2.b){ //Reverse Intake
                intakeState = -1;
            }else{
                intakeState = 0;
            }

            if (gamepad2.x){ //Outtake
                outtakeState = 1;
            }else if (gamepad2.a){ //Reverse Outtake
                outtakeState = -1;
            }else{
                outtakeState = 0;
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
        public double getIntakeAngle(){
            double PNPVoltage = pivotIntakePNP.getVoltage();

            return (degpervoltage*PNPVoltage);
        }

        public void GoToAngle(double angle){
            intakeTargetPos = angle;

            double currentPos = getIntakeAngle();
            double pid = intakePController.calculate(currentPos, intakeTargetPos);

            intakePivotL.setPower(pid);
            intakePivotR.setPower(pid);
        }

    }
}



