package frc.robot.commands.Arcade;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class CMD_TeleopDrive extends CommandBase {
  private final SUB_Drivetrain drivetrain;
  private Supplier<Double> speedLeft, speedRight;

  public CMD_TeleopDrive(SUB_Drivetrain drive, Supplier<Double> L, Supplier<Double> R){
    this.drivetrain = drive;
    this.speedLeft = L;
    this.speedRight = R;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    //move the robot with the arcade drive
    drivetrain.setMotorsArcade(speedLeft.get()*0.5, speedRight.get()*0.5);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}