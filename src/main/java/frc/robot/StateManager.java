package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Gripper;
import frc.robot.subsystems.SUB_Tower;

public class StateManager extends SubsystemBase{

    Gamepiece gp = Gamepiece.cone;
    final Field2d field2d = RobotContainer.field2d;
    final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;

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

    public void toggleGP(){
        if(gp.equals(Gamepiece.cone)){
            gp = Gamepiece.cube;
        }else{
            gp = Gamepiece.cone;
        }
    }

    //VALUES FOR CUBES ARE ALL ARBITRARY CURRENTLY

    //Arm
    public double kScoringPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kScoringConePosition;
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