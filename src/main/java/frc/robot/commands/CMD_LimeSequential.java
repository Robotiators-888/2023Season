package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CMD_LimeAlign;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.commands.CMD_LimeDrive;
import frc.robot.subsystems.SUB_Limelight;
// We're doing sequential in order to algin then drive towards it
public class CMD_LimeSequential extends SequentialCommandGroup {
  public CMD_LimeSequential(SUB_Drivetrain Drivetrain, SUB_Limelight Limelight) {  
    // Turns on the light
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(3); 
      }, Limelight));
    // Waits for the limelight to turn on and also due to latency
    addCommands(new WaitCommand(2));
    // Starts to align towards it
    addCommands(new CMD_LimeAlign(Limelight, Drivetrain));
    // Starts to drive towards it
    addCommands(new CMD_LimeDrive(Limelight,Drivetrain));
  }
} 