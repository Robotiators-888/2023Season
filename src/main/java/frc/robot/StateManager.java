package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase{

    Gamepiece gp = Gamepiece.cone;

    public StateManager(){
    }

    enum Gamepiece {
        cube,
        cone
    }

   /**
    * Sets state to Cube
    * 
    * @param setCube
    */

    public void setCube(){
        gp = Gamepiece.cube;
    }

   /**
    * Sets state to Cone
    * 
    * @param setCone
    */

    public void setCone(){
        gp = Gamepiece.cone;
    }

    //VALUES FOR CONES ARE ALL ARBITRARY CURRENTLY

    //Arm
    public double kScoringPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kScoringPosition;
        }
        else {
           return 0;
        }
    }

    //Gripper
    public double kClosePosition(){
        if (gp == Gamepiece.cone) {
            //for cones
            return Constants.Gripper.kCloseConePosition;
        }
        else {
            return Constants.Gripper.kCloseCubePosition;
        }
    }

    public double kPosition(){
        if (gp == Gamepiece.cone) {
            //for cones
            return Constants.Gripper.kConePosition;
        }
        else {
           return Constants.Gripper.kCubePosition;
        }
    }
    
}
