package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LimeAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.LimeDrive;
import frc.robot.subsystems.Limelight;
public class LimeSequential extends SequentialCommandGroup {
  public LimeSequential(Drivetrain Drivetrain, Limelight Limelight) {  
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(3); 
      }, Limelight));
    addCommands(new WaitCommand(2));
    addCommands(new LimeAlign(Limelight, Drivetrain));
    addCommands(new LimeDrive(Limelight,Drivetrain));
  }
} 