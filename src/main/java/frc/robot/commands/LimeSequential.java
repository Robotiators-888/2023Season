package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LimeAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.LimeDrive;
import frc.robot.subsystems.Limelight;
// We're doing sequential in order to algin then drive towards it
public class LimeSequential extends SequentialCommandGroup {
  public LimeSequential(Drivetrain Drivetrain, Limelight Limelight) {  
    // Turns on the light
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(3); 
      }, Limelight));
    // Waits for the limelight to turn on and also due to latency
    addCommands(new WaitCommand(2));
    // Starts to align towards it
    addCommands(new LimeAlign(Limelight, Drivetrain));
    // Starts to drive towards it
    addCommands(new LimeDrive(Limelight,Drivetrain));
  }
} 