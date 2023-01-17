// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Drivetrain extends SubsystemBase {
  /** Creates a new SUB_Drivetrain. */
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.ID_LEFT_PRIMARY_DRIVE, MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.ID_LEFT_SECONDARY_DRIVE, MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.ID_RIGHT_PRIMARY_DRIVE, MotorType.kBrushless);
  private CANSparkMax rightSecondary = new CANSparkMax(Constants.ID_RIGHT_SECONDARY_DRIVE, MotorType.kBrushless);

  MotorControllerGroup leftGroup = new MotorControllerGroup(leftPrimary, leftSecondary);
  MotorControllerGroup righGroup = new MotorControllerGroup(rightPrimary, rightSecondary);
  
  
  AHRS navx = new AHRS(SerialPort.Port.kMXP);

  private RelativeEncoder leftEncoder = leftPrimary.getEncoder();
  private RelativeEncoder righEncoder = rightPrimary.getEncoder();

  public SUB_Drivetrain() {

    leftPrimary.setInverted(true);
    leftSecondary.setInverted(true);
    

    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
