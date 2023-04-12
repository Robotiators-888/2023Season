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
    private double lastAngle = 0;
    private double drivePower;
    private double ForwardMult = 1.5; // must have its own max speed
    private double maxSpeed = 0.5;
    private double diferenceInAngle;
    double stopAngle = 10.0;
    boolean driveBackwards;

    //Limits when the robot should stop on the charge station going towards the drivers
    private double forwardLimit = 10.5;

    // Limits when the robot should stop on the charge station going away from the drivers
    private double backwardLimit = -6;

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
  public void execute() { SmartDashboard.putNumber("AutoBalanceStopAngle", stopAngle);

this.currentAngle = m_drivetrain.getPitch();
if(currentAngle > 12.5 ){
  m_drivetrain.driveArcade(0.325, 0);

}else if(currentAngle < -7){
  m_drivetrain.driveArcade(-0.25, 0);

}else if(currentAngle <= 12.5 && currentAngle >= -7){
  m_drivetrain.driveArcade(0.0, 0);

}
 //m_drivetrain.driveArcade(0.3, 0);

 //System.out.println("drivePower*FM: "+(drivePower*ForwardMult)+"angle: "+currentAngle+" ForwardMult:"+ForwardMult+" difInAngle: "+diferenceInAngle+" maxSpeed: "+maxSpeed);
 SmartDashboard.putNumber("drivePower*FM", (drivePower*ForwardMult));
 SmartDashboard.putNumber("pitch balance angle",currentAngle);
 SmartDashboard.putNumber("ForwardMult",ForwardMult);
 SmartDashboard.putNumber("difInAngle", diferenceInAngle);
 SmartDashboard.putNumber("AutoBalance maxSpeed", maxSpeed);
 SmartDashboard.putBoolean("balancing", true);
 
 this.lastAngle = currentAngle;
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
