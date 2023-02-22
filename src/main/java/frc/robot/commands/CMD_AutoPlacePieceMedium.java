package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Gripper;
import frc.robot.subsystems.SUB_Tower;

public class CMD_AutoPlacePieceMedium extends SequentialCommandGroup {
    public CMD_AutoPlacePieceMedium(SUB_Gripper Gripper, SUB_Tower Tower){
      new InstantCommand(() -> Tower.setTargetPosition(Constants.Arm.kScoringPosition, Gripper));
      addCommands(new InstantCommand(() -> {Gripper.openGripper();SmartDashboard.putNumber("Gripper Status", Gripper.getSetPosition());}));
    }
}