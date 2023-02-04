// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Gripper;

public class CMD_GripperRun extends CommandBase {

  SUB_Gripper sub_Gripper;

  /** Creates a new GripperCommand. */
  public CMD_GripperRun(SUB_Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    sub_Gripper = gripper;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub_Gripper.speedSet(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub_Gripper.speedSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}