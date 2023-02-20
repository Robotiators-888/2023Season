// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax.IdleMode;

public class SUB_Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.ID_LEFT_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.ID_LEFT_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.ID_RIGHT_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSecondary = new CANSparkMax(Constants.ID_RIGHT_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);

// create a speed controller group for each side
private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

// create a drive train group with the speed controller groups
private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);
 
private double prevXAccel = 0;
private double prevYAccel = 0;
private double XAccel = 0;
private double YAccel = 0;

Accelerometer accelerometer = new BuiltInAccelerometer();

public SUB_Drivetrain() {
    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);
    leftPrimary.setInverted(true);
    leftSecondary.setInverted(true);

    
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
    double xAccel = accelerometer.getX();
    double yAccel = accelerometer.getY();

    this.XAccel = xAccel;
    this.YAccel = yAccel;
    prevXAccel = xAccel;
    prevYAccel = yAccel;

    SmartDashboard.putNumber("XAccelJerk",getXAccelJerk() );
    SmartDashboard.putNumber("YAccelJerk",getYAccelJerk() );

  }

  public void setMotorsArcade(double xSpeed,double zRotation){
    driveTrain.arcadeDrive(xSpeed, zRotation);
    SmartDashboard.putNumber("drivetrain xspeed: ", xSpeed);
    SmartDashboard.putNumber("drivetrain rotation: ", zRotation);
  }



  public double getXAccelJerk(){
    // Calculates the jerk in the X and Y directions
      // Divides by .02 because default loop timing is 20ms
      double xJerk = (XAccel - prevXAccel)/.02;
     return xJerk;
  }
  
  public double getYAccelJerk(){
    // Calculates the jerk in the X and Y directions
      // Divides by .02 because default loop timing is 20ms
      double yJerk = (YAccel - prevYAccel)/.02;
     return yJerk;
  }
  
}