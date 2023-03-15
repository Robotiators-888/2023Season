package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.Autonomous;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.StateManager;

// We're doing sequential in order to algin then drive towards it
public class CMD_AprilSequential extends SequentialCommandGroup {
  final SUB_Gripper gripper = RobotContainer.gripper;
  final SUB_AprilTag aprilTag = RobotContainer.apriltag;
  final SUB_Tower tower = RobotContainer.tower;
  final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
  final StateManager stateManager = RobotContainer.stateManager;
  final Autonomous Autonomous = new frc.robot.Autonomous();
  public CMD_AprilSequential() {
    // Drivers get a quick way to know april tag sequence is working
  }
    public Command limelightPlacement(){
      return new SequentialCommandGroup(
        new InstantCommand(() -> {stateManager.setCube();}, stateManager),
        new RunCommand(() -> {aprilTag.switchapipeline(1);}, aprilTag),
        new RunCommand(() -> {aprilTag.aprilAlign();}, aprilTag).until(() -> (aprilTag.getX() <= 0.05)),
        new InstantCommand(() -> {drivetrain.setBrakeMode(true);}, drivetrain),
        new RunCommand(() -> {aprilTag.aprilDrive();}, aprilTag).until(() -> (aprilTag.getDistance() <= 12)),
        new InstantCommand(() -> {drivetrain.setBrakeMode(true);}, drivetrain),
        new RunCommand(() -> {aprilTag.aprilAlign();}, aprilTag).until(() -> (aprilTag.getX() <= 0.05)),
        new InstantCommand(() -> {drivetrain.setBrakeMode(true);}, drivetrain),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringConePosition, tower)),
         new SequentialCommandGroup(
          new WaitCommand(2.5), 
          new InstantCommand(() -> gripper.openGripper(), gripper))),
          new SequentialCommandGroup(
            new WaitCommand(1), 
            new InstantCommand(()->gripper.closeGripper()))),
            new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)));
    }
} 