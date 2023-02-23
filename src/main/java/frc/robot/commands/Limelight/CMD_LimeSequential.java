package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.CMD_AutoPlacePieceMedium;
import frc.robot.commands.Limelight.*;

// We're doing sequential in order to algin then drive towards it
public class CMD_LimeSequential extends SequentialCommandGroup {
  public CMD_LimeSequential(SUB_Drivetrain Drivetrain, SUB_Limelight Limelight, SUB_Gripper Gripper, SUB_Tower Tower) {
    // Drivers get a quick way to know april tag sequence is working
    SmartDashboard.putBoolean("limelight sequence", true);
    // Switches the pipeline to the correct pipeline
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
    // Realignts to remove error
    addCommands(new CMD_LLAlign(Limelight, Drivetrain));
    // Turns off the light
    addCommands(Commands.runOnce(
      () -> {
        Limelight.setLed(1);
      }, Limelight));
      SmartDashboard.putBoolean("limelight sequence", false);
    // Places the piece to medium node
      addCommands(new CMD_AutoPlacePieceMedium(Gripper, Tower));
    }

    
} 