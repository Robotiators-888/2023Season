package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.SUB_Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_TeleDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SUB_Drivetrain drive;
  private Supplier<Double> Left, Right;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CMD_TeleDrive(SUB_Drivetrain drivetrain, Supplier<Double> L, Supplier<Double> R) {
    this.drive = drivetrain;
    this.Left = L;
    this.Right = R;
  
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      drive.setMotorsArcade(Left.get()*0.5,Right.get()*0.5);
      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setBrakeMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}