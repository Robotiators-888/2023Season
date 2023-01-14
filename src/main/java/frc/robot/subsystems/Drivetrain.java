// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.ID_LEFT_PRIMARY,
  CANSparkMaxLowLevel.MotorType.kBrushless);
private CANSparkMax leftSecondary = new CANSparkMax(Constants.ID_LEFT_SECONDARY,
CANSparkMaxLowLevel.MotorType.kBrushless);
private CANSparkMax rightPrimary = new CANSparkMax(Constants.ID_RIGHT_PRIMARY,
CANSparkMaxLowLevel.MotorType.kBrushless);
private CANSparkMax rightSecondary = new CANSparkMax(Constants.ID_RIGHT_SECONDARY,
CANSparkMaxLowLevel.MotorType.kBrushless);

// create a speed controller group for each side
private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

// create a drive train group with the speed controller groups
private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);
  public Drivetrain() {
    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);
    leftPrimary.setInverted(true);
    leftSecondary.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorsArcade(double xSpeed,double zRotation){
    driveTrain.arcadeDrive(xSpeed, zRotation*-1);
  }
}
