package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipSub;

public class towerup extends CommandBase{
    public manipSub manip;
    
    public towerup(manipSub manip){
        this.manip = manip;
        addRequirements(manip);
    }

    // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    manip.towerMoveDown();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    manip.towerStop();
  }

}
