package frc.robot.commands;
import frc.robot.subsystems.SUB_Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class CMD_AutoDrive extends CommandBase {
  //creates driveSubsystem object from the DriveSubsystem class/file
  private final SUB_Drivetrain drivetrain;

  public CMD_AutoDrive(SUB_Drivetrain drive){
    this.drivetrain = drive;

    addRequirements(drive);
  }
  
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivetrain.setBrakeMode(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Go forward at 0.45 speed
    drivetrain.setMotorsArcade(Constants.AUTO_SPEED, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setBrakeMode(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}