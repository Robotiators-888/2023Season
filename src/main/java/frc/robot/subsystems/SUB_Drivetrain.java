// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, righGroup);
  
  AHRS navx = new AHRS(SerialPort.Port.kMXP);

  private RelativeEncoder leftEncoder = leftPrimary.getEncoder();
  private RelativeEncoder righEncoder = rightPrimary.getEncoder();

  public SUB_Drivetrain() {
    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);
    leftPrimary.setInverted(true);
    leftSecondary.setInverted(true);

    leftPrimary.setSmartCurrentLimit(60);
    leftSecondary.setSmartCurrentLimit(60);
    rightPrimary.setSmartCurrentLimit(60);
    rightSecondary.setSmartCurrentLimit(60);
  }

  public void setBrakeMode(boolean brake){
    if(brake){
        leftPrimary.setIdleMode(IdleMode.kBrake);
        leftSecondary.setIdleMode(IdleMode.kBrake);
        rightPrimary.setIdleMode(IdleMode.kBrake);
        rightSecondary.setIdleMode(IdleMode.kBrake);
    }else{
        leftPrimary.setIdleMode(IdleMode.kCoast);
        leftSecondary.setIdleMode(IdleMode.kCoast);
        rightPrimary.setIdleMode(IdleMode.kCoast);
        rightSecondary.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorsArcade(double xSpeed,double zRotation){
    driveTrain.arcadeDrive(xSpeed, zRotation);
  }

  public void setMotorsTank(double leftSpeed, double rightSpeed, double Speed) {
    driveTrain.tankDrive(leftSpeed * Speed, rightSpeed * Speed);
  }

  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoder(){
    return righEncoder.getPosition();
  }

  public void resetEncoders(){
    leftPrimary.getEncoder().setPosition(0);
    rightPrimary.getEncoder().setPosition(0);
  }

  public double getYaw(){
    return navx.getYaw();
  }

  public double getPitch(){
    return navx.getPitch();
  }

  public double getRoll(){
    return navx.getRoll();
  }
}
