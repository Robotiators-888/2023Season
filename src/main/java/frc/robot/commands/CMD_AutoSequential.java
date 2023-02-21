package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class CMD_AutoSequential extends SequentialCommandGroup {
    public CMD_AutoSequential(SUB_Gripper Gripper, SUB_Tower Tower, SUB_Drivetrain Drivetrain){
      addCommands(new InstantCommand(() -> Tower.setTargetPosition(Constants.Arm.kScoringPosition, Gripper)));
      new WaitCommand(1.5);
      addCommands(new InstantCommand(() -> {Gripper.openGripper();SmartDashboard.putNumber("Gripper Status", Gripper.getSetPosition());}));
      new WaitCommand(1.5);
      addCommands(new InstantCommand(() -> {Gripper.closeGripper();SmartDashboard.putNumber("Gripper Status", Gripper.getSetPosition());}));
      new WaitCommand(1.5);
      addCommands(new InstantCommand(() -> Tower.setTargetPosition(Constants.Arm.kHomePosition, Gripper)));
      new WaitCommand(1.5);
      addCommands(new RunCommand(() -> {Drivetrain.driveArcade(0.06,0.0);}).withTimeout(3));

    }
}