package org.firstinspires.ftc.teamcode.Subsystems;

/* This is a singleton class created to store and then read back the state of certain variables
14133 intends to use it to store the "Alliance" variable during match setup and then carry the variable to teleop
Singletons create a single instance of themselves and prevent any other instances from being created
 */
public class AllianceSingleton {
    private static AllianceSingleton AllianceSelector = null; //Creates a single instance of AllianceSingleton
    public Boolean Alliance; //Variable to store the alliance the robot is on
    private AllianceSingleton() {
        Alliance = null;
    }
    public static AllianceSingleton AllianceInstance() { //Function alliance instance, which prevents another instance to be created
        if (AllianceSelector == null) {
            AllianceSelector = new AllianceSingleton();
        }
        return AllianceSelector;
    }
    public void SetAlliance(Boolean AllianceInput){ //Sets the variable "Alliance" to a new value
        Alliance = AllianceInput;

    }
    public Boolean GetAlliance(){ //Gets the current value of alliance
       return Alliance;
    }
}
