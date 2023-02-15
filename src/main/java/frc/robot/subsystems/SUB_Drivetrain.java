// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//navx
// import com.kauailabs.navx.frc.AHRS;
public class SUB_Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  // Gets the motors
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.ID_LEFT_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.ID_LEFT_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.ID_RIGHT_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSecondary = new CANSparkMax(Constants.ID_RIGHT_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);

  // create a speed controller group for each side
  private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
  private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

  // create a drive train group with the speed controller groups
  private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);

  //navx
  // private AHRS navx = new AHRS();

  private Joystick controller1 = new Joystick(Constants.JOYSTICK_PORT);

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
    /* 
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
    */
  }

  // public void putNumber(int num) {
  //   // This method will be called once per scheduler run
  //   SmartDashboard.putNumber("Drive Mode", num); 
  // }

  // The different drivetrains
  public void setMotorsArcade(double xSpeed,double zRotation){
    driveTrain.arcadeDrive(xSpeed, zRotation);
  }

  public void setMotorsTank(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void setMotorsCurvature(double xSpeed, double zRotation, boolean isQuickTurn){
    driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  // Encoders getting position
  public double getLeftEncoder(){
    return leftPrimary.getEncoder().getPosition();
  }

  public double getRightEncoder(){
    return rightPrimary.getEncoder().getPosition();
  }

  // public double getAngle(){
  //   return navx.getAngle();
  // }

  // public void resetAngle(){
  //   navx.reset();
  // }

  // public double getYaw(){
  //   return navx.getYaw();
  // }

  // public double getPitch(){
  //   return navx.getPitch();
  // }

  // public double getRoll(){
  //   return navx.getRoll();
  // }

  // Gets the number from the smart dashboard to change drive
  // public int driveMode(){
  //   return (int) SmartDashboard.getNumber("Drive Mode", 0);
  // }

  // Switches it?
}
