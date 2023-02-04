// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Gripper extends SubsystemBase {
  CANSparkMax GripperSparkMax = new CANSparkMax(Constants.GRIPPER_SPARKMAX_CANID, MotorType.kBrushless);

  /** Creates a new GripperSubsystem. */
  public SUB_Gripper() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /**
   * Sets speed of GripperSparkMax
   * 
   * @param speed double speed of motor [-1.0 to 1.0]
   */
  public void speedSet(double speed) {
    GripperSparkMax.set(speed);
  }
}
