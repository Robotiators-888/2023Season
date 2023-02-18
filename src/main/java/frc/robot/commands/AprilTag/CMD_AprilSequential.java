package frc.robot.commands.AprilTag;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AprilTag.*;
import frc.robot.subsystems.*;

// We're doing sequential in order to algin then drive towards it
public class CMD_AprilSequential extends SequentialCommandGroup {
  public CMD_AprilSequential(SUB_Drivetrain Drivetrain, SUB_AprilTag aprilTag) {  
    addCommands(Commands.runOnce(
      () -> {
        aprilTag.switchapipeline(0);; 
      }, aprilTag));
    // Starts to align towards it
    addCommands(new CMD_ATAlign(aprilTag, Drivetrain));
    // Starts to drive towards it
    addCommands(new CMD_ATDrive(aprilTag, Drivetrain));

    addCommands(new CMD_ATAlign(aprilTag, Drivetrain));
  }
} 