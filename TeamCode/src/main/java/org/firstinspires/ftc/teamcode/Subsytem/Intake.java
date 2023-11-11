package org.firstinspires.ftc.teamcode.Subsytem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    // Instantiate the motor variables
    static DcMotorEx intakeSucker;
    static CRServo intakePivot;
    AnalogInput pivotIntakePNP;

    private PIDController controller; // todo: make ff
    public static double p = 0, i = 0, d = 0; // todo: tune intake pivot pid

    //boolean Possession = true; //Variable telling whether we have possession of a game piece or not

    int teleopState = 0;

    final double suckerSpeed = 1; // todo: figure out good speed
    final double degpervoltage = 270/3.3;

    double  intakeTargetPos = 0; // todo: see what starting pos we want
    public Catcher objcatcher;
    public Pivot objpivot;

    public Intake(HardwareMap hardwareMap){                 // Motor Mapping
        intakeSucker = hardwareMap.get(DcMotorEx.class, "intakeSuck");
        intakePivot = hardwareMap.get(CRServo.class, "intakePiv");
        pivotIntakePNP = hardwareMap.get(AnalogInput.class, "intakePNP");

        controller.setPID(p, i, d);

        objcatcher = new Catcher();
        objpivot = new Pivot();


    }

    public class Catcher {
        private void runIntake(double intakeState){ // 1 is intake, 0 is off, -1 is outtake
            intakeSucker.setPower(suckerSpeed *intakeState);
        }

        public void Teleop(Gamepad gamepad2, Telemetry telemetry) { //Code to be run in Op Mode void Loop at top level

            if (gamepad2.a){ //Intake
                teleopState = 1;
            }else if (gamepad2.b){ //Outtake
                teleopState = -1;
            }else{
                teleopState = 0;
            }

            runIntake(teleopState);
            telemetry.addData("teleopState", teleopState);
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

            return degpervoltage*PNPVoltage;
        }

        public void GoToAngle(double angle){
            intakeTargetPos = angle;

            double currentPos = getIntakeAngle();
            double pid = controller.calculate(currentPos, intakeTargetPos);

            intakePivot.setPower(pid);
        }
    }
}



