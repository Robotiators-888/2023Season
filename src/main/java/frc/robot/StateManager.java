package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Gripper;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Tower;

public class StateManager extends SubsystemBase{

    Gamepiece gp;
    boolean haveGP = false;
    final Field2d field2d = RobotContainer.field2d;
    //final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;
    final SUB_Blinkin blinkin = RobotContainer.blinkin;
    final SUB_Roller roller = RobotContainer.roller;

    public StateManager(){
        gp = Gamepiece.cone;
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
        blinkin.solidViolet();
    }

   /**
    * Sets state to Cone
    * 
    * @param setCone
    */

    public void setCone(){
        gp = Gamepiece.cone;
        blinkin.solidOrange();
    }

    public void toggleGP(){
        if(gp.equals(Gamepiece.cone)){
            gp = Gamepiece.cube;
            blinkin.solidViolet();
        }else{
            gp = Gamepiece.cone;
            blinkin.solidOrange();
        }
    }

    //VALUES FOR CUBES ARE ALL ARBITRARY CURRENTLY

    //Arm
    public double kScoringPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kScoringHighCone;
        }
        else {
           return Constants.Arm.kScoringHighCube;
        }
    }

    //Gripper in case the roller fails
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

    public void intakeRoller(){
        if(gp.equals(Gamepiece.cone)){ 
            roller.driveRoller(0.50);
        }else{
            roller.driveRoller(-0.50);
        }
    }

    public void outtakeRoller(){
        if(gp.equals(Gamepiece.cone)){ 
            roller.driveRoller(-0.50);
        }else{
            roller.driveRoller(0.50);
        }
    }

    public void stopRoller(){
        roller.driveRoller(0.0);
    }

    public void grabGP(){
        haveGP = true;
    }

    public void dropGp(){
        haveGP = false;
    }

    public boolean securedGP(){
        return haveGP;
    }
    
}
