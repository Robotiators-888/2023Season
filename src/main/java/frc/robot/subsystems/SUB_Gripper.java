// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.PIDGains;
import frc.robot.Constants;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SUB_Gripper extends SubsystemBase {

  private CANSparkMax GripperSparkMax;
  private AbsoluteEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;
  private double m_prevSetpoint;

  DataLog log = DataLogManager.getLog();
  DoubleLogEntry gripperMotorOutput = new DoubleLogEntry(log, "/gripper/motorOutput");

  /** Creates a new GripperSubsystem. */
  public SUB_Gripper() {
    GripperSparkMax = new CANSparkMax(Constants.Gripper.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    GripperSparkMax.restoreFactoryDefaults();
    GripperSparkMax.setInverted(false);
    GripperSparkMax.setSmartCurrentLimit(Constants.Gripper.kCurrentLimit);
    //GripperSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    //GripperSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    GripperSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Gripper.kSoftLimitForward);
    GripperSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Gripper.kSoftLimitReverse);

    // get absolute encoder connect to spark max
    m_encoder = GripperSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    
    GripperSparkMax.burnFlash();

    m_setpoint = Constants.Gripper.kCloseConePosition;
  }

  public boolean isSafe() {
    return m_encoder.getPosition() > Constants.Gripper.kSafePosition;
  }

  public void openConeGripper() {
    SmartDashboard.putNumber("Gripper Status", getSetPosition());
    m_setpoint = Constants.Gripper.kConePosition;
  }

  public void closeConeGripper() {
    SmartDashboard.putNumber("Gripper Status", getSetPosition());
    m_setpoint = Constants.Gripper.kCloseConePosition;
  }

  public void openCubeGripper() {
    SmartDashboard.putNumber("Gripper Status", getSetPosition());
    m_setpoint = Constants.Gripper.kCubePosition;
  }

  public void closeCubeGripper() {
    SmartDashboard.putNumber("Gripper Status", getSetPosition());
    m_setpoint = Constants.Gripper.kCloseCubePosition;
  }
  public double getSetPosition(){
    return m_setpoint;

  }

  /**
   * Sets speed of GripperSparkMax
   * 
   * @param speed double speed of motor [-1.0 to 1.0]
   */

  public void driveGripper(double speed) {
    GripperSparkMax.set(speed);
  }

  @Override
  public void periodic() {
      // code to drive gripper to setpoint
      // if (m_encoder.getAbsolutePosition() > m_setpoint){
      //   driveGripper(-0.1);
      // }
      // else{
      //   driveGripper(0.1);
      // }

      // print absolute position of gripper
      SmartDashboard.putNumber("absolute encoder position", m_encoder.getPosition());
  }

  

}
