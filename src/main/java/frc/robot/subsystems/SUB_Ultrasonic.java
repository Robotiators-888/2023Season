// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.libs.RunningAverageQueue;

public class SUB_Ultrasonic extends SubsystemBase {

  private final AnalogInput ultrasonic = new AnalogInput(0);
  private double rawValue;
  private double avgDistance;
  private RunningAverageQueue runningAvgObject;
  
  /** Creates a new ExampleSubsystem. */
  public SUB_Ultrasonic() {
    runningAvgObject = new RunningAverageQueue(5);
  }

  public void updateRaw() {
    rawValue = ultrasonic.getValue();
  }

  /**
   * gets distance to closest object
   *
   * @return distance (in inches) (10 feet max) (12 inch min)
   */
  public double getDistance() {
    // Inline construction of command goes here.
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    updateRaw();
    double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;

    return Units.metersToInches(currentDistanceCentimeters/100);
  }

  /**
   * gets avg distance to closest object over past 5 scheduler runs
   * 
   * @return avergage distance over past 5 scheduler runs
   */
  public double getAvgDistance() {
    avgDistance = runningAvgObject.getRunningAverage();
    return avgDistance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runningAvgObject.insert(getDistance());
    updateRaw();
    SmartDashboard.putNumber("Ultrasonic Distance Inches",getDistance());
    SmartDashboard.putNumber("Ultrasonic Running Avg Distance Inches",getAvgDistance());
    SmartDashboard.putBoolean("stop feederstation",getDistance()<45);
    SmartDashboard.putBoolean("ultrasonic detected",getDistance()<150);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}