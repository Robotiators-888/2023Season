package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase{

    public StateManager(){
    }

    boolean cube_or_cone = true;

    /**
   * Toggles Cube or Cone mode
   * 
   * @param cube_or_cone true for cone, false for cube
   */


    /**
   * Sets Cube or Cone mode
   * 
   * @param cube_or_cone true for cone, false for cube
   */

    public void setCubeCone(boolean b){
        cube_or_cone = b;
    }

    //VALUES FOR CONES ARE ALL ARBITRARY CURRENTLY

    //Arm
    public double kScoringPosition(){
        if (cube_or_cone == true) {
            //for cones
            return Constants.Arm.kScoringPosition;
        }
        else {
           return 0;
        }
    }

    //Gripper
    public double kClosePosition(){
        if (cube_or_cone == true) {
            //for cones
            return Constants.Gripper.kCloseConePosition;
        }
        else {
            return Constants.Gripper.kCloseCubePosition;
        }
    }

    public double kPosition(){
        if (cube_or_cone == true) {
            //for cones
            return Constants.Gripper.kConePosition;
        }
        else {
           return Constants.Gripper.kCubePosition;
        }
    }
    
}
