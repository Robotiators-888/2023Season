// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class UpAndOverBalance extends CommandBase {
  /** Creates a new ReverseBalance. */
  SUB_Drivetrain m_drivetrain;

    private double currentAngle;
    
  public UpAndOverBalance(SUB_Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("BALANCING", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

this.currentAngle = m_drivetrain.getPitch();
if(currentAngle > 12.5 ){
  m_drivetrain.driveArcade(0.325, 0);

}else if(currentAngle < -7){
  m_drivetrain.driveArcade(-0.25, 0);

}else if(currentAngle <= 12.5 && currentAngle >= -7){
  m_drivetrain.driveArcade(0.0, 0);

}

 
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("BALANCING", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
