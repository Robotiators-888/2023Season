package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Limelight.*;

// We're doing sequential in order to algin then drive towards it
public class CMD_LimeSequential extends SequentialCommandGroup {
  public CMD_LimeSequential(SUB_Drivetrain Drivetrain, SUB_Limelight Limelight) {
    SmartDashboard.putBoolean("limelight sequence", true);
    addCommands(Commands.runOnce(
      () -> {
        Limelight.switchapipeline(1); 
      }, Limelight));
    // Turns on the light
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(3); 
      }, Limelight));
    // Waits for the limelight to turn on and also due to latency
    addCommands(new WaitCommand(1));
    // Starts to align towards it
    addCommands(new CMD_LLAlign(Limelight, Drivetrain));
    // Starts to drive towards it
    addCommands(new CMD_LLDrive(Limelight, Drivetrain));

    addCommands(new CMD_LLAlign(Limelight, Drivetrain));
    // Turns off the light
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(1);
      }, Limelight));
      SmartDashboard.putBoolean("limelight sequence", false);
    }

} 