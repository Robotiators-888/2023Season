package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimeSequential extends SequentialCommandGroup {
  public LimeSequential(Drivetrain Drivetrain, Limelight Limelight) {
    addCommands(new LimeAlign(Limelight, Drivetrain));
    addCommands(new LimeDrive(Limelight,Drivetrain));
  }
} 