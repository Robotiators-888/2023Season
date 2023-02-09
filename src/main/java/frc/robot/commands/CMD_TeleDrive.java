package frc.robot.commands;

import java.util.function.Supplier;

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
    // gets the drivetrain subsystem and the joystick values
    this.drive = drivetrain;
    this.Left = L;
    this.Right = R;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //Uses the arcade drive method from the drivetrain subsystem
      drive.setMotorsArcade(Left.get()*0.5,Right.get()*0.5);
      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // makes sure the motors are off when the command is finished
    drive.setBrakeMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}