package org.firstinspires.ftc.teamcode.Subsytem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    DcMotorEx slideM;
    DcMotorEx pivotArmLM, pivotArmRM;

    AnalogInput pivotArmPNP;

    private PIDController controller; // todo: make ff
    public static double p = 0, i = 0, d = 0; // todo: tune arm pivot pid

    boolean togglePivotArm = true;
    int pivotArmPos = -1;

    int pivotArmMax = 3;
    int pivotArmMin = -1;

    public Arm(HardwareMap hardwareMap){
        slideM = hardwareMap.get(DcMotorEx.class, "slideM");

        pivotArmLM = hardwareMap.get(DcMotorEx.class, "PivotArmLM");
        pivotArmRM = hardwareMap.get(DcMotorEx.class, "PivotArmRM");

        pivotArmPNP = hardwareMap.get(AnalogInput.class, "intakePNP");

        controller.setPID(p, i, d);
    }

    public class Pivot {
        public void Teleop(Gamepad gamepad2, Telemetry telemetry){
            if (togglePivotArm && (gamepad2.dpad_up || gamepad2.dpad_down)) {  // Only execute once per Button push
                togglePivotArm = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (gamepad2.dpad_down) {  // If the d-pad up button is pressed
                    pivotArmPos = pivotArmPos + 1; //Increase Arm position
                    if (pivotArmPos > pivotArmMax) { //If arm position is above 3
                        pivotArmPos = pivotArmMax; //Cap it at 3
                    }
                } else if (gamepad2.dpad_up) { // If d-pad down button is pressed
                    pivotArmPos = pivotArmPos - 1; //Decrease arm position
                    if (pivotArmPos < pivotArmMin) { //If arm position is below -3
                        pivotArmPos = pivotArmMin; //cap it at -3
                    }
                }

            }
            else if (!gamepad2.dpad_up && !gamepad2.dpad_down) { //if neither button is being pressed
                togglePivotArm = true; // Button has been released, so this allows a re-press to activate the code above.
            }

            GoToPosition(pivotArmPos);
        }

        //public void GoToPosition(int position)
    }

}
