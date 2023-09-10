
package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.AllianceSingleton;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Lights;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode") // Labels program in Driver station Selection

//This is a test
//My favorite shape is a nonagon
//I like to ride dirt bikes RS

//TEST Commit Vihaan

//COMMENT YOUR CODE (VIHAAN)! We're adding a lot of automation, which is nice, but it is going to be REALLY difficult to troubleshoot.

public class  FTC_14133_2022 extends OpMode {
    private Drivetrain drivetrain=null;
    private Intake Intake=null;
    private Lift Lift=null;
    private Sensors Sensors=null;
    private Lights Lights=null;
    boolean [] switches;
    boolean Alliance;
    boolean Warehouse_TurnTable;

 public void init() {
     drivetrain = new Drivetrain(hardwareMap);
     Intake = new Intake(hardwareMap);
     Lift = new Lift(hardwareMap);
     Sensors = new Sensors(hardwareMap);
     Lights = new Lights(hardwareMap);
     //Alliance = AllianceSingleton.AllianceInstance().GetAlliance();
 }

 public void start() {
     telemetry.addData("Status", "Start");
     telemetry.update();
 }


 public void loop() {
     telemetry.addData("Status", "Looping");

     Lift.Teleop(gamepad2, telemetry); //Run the regular arm function

     drivetrain.Teleop(gamepad1, telemetry, Lift.GetArmHome(), Lift.GetElevatorHome());

     Lights.Update_Lights(Intake.getPossession(), Alliance, gamepad1.left_stick_button || gamepad2.left_stick_button);

     Intake.Teleop(gamepad2, telemetry); //Passes position of the arm so intake direction can change.
     Intake.beambreak_print(telemetry);
     telemetry.update();
     Intake.Possession_Check();

 }

/*    public void stop() {
        Lift.GotoPosition(4 , 0, 0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        super.stop();
    }*/


}
