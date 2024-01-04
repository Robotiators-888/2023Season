package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Tower;

public class StateManager extends SubsystemBase{

    Gamepiece gp;
    boolean haveGP = false;
    final Field2d field2d = RobotContainer.field2d;
    //final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = null;
    final SUB_Blinkin blinkin = null;
    final SUB_Roller roller = null;

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
        SmartDashboard.putBoolean("Robot State", true);
        blinkin.solidViolet();
    }

   /**
    * Sets state to Cone
    * 
    * @param setCone
    */

    public void setCone(){
        gp = Gamepiece.cone;
        SmartDashboard.putBoolean("Robot State", false);
        blinkin.solidOrange();
    }

    public void toggleGP(){
        if(gp.equals(Gamepiece.cone)){
            gp = Gamepiece.cube;
            SmartDashboard.putBoolean("Robot State", true);
            blinkin.solidViolet();
        }else{
            gp = Gamepiece.cone;
          
            SmartDashboard.putBoolean("Robot State", false);
            blinkin.solidOrange();
        }
    }

    public Gamepiece getState(){
        return gp;
    }

    //VALUES FOR CUBES ARE ALL ARBITRARY CURRENTLY

    //Arm
    public double kScoringPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kScoreConePosition;
        }
        else {
           return Constants.Arm.kScoreCubePosition;
        }
    }

    public double kGroundPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kGroundConePosition;
        }
        else {
           return Constants.Arm.kGroundCubePosition;
        }
    }
    public double kFeederPosition(){
        if (gp == Gamepiece.cone) {
            return Constants.Arm.kFeederConePosition;
        }
        else {
           return Constants.Arm.kFeederCubePosition;
        }
    }
    public double kHomePosition(){
        return Constants.Arm.kHomePosition;
    }

    public void intakeRoller(){
        if(gp.equals(Gamepiece.cone)){ 
            roller.driveRoller(0.65);
        }else{
            roller.driveRoller(-0.50);
        }
    }

    public void outtakeRoller(){
        if(gp.equals(Gamepiece.cone)){ 
            roller.driveRoller(-0.65);
        }else{
            roller.driveRoller(0.50);
        }
    }

    public void stopRoller(){
        if(gp.equals(Gamepiece.cone)){ 
            roller.driveRoller(0.0);
        }else{
            roller.driveRoller(-0.0);
        }
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
