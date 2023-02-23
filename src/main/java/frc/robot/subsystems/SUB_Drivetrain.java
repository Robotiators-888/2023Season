// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//navx
// import com.kauailabs.navx.frc.AHRS;
public class SUB_Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  // Gets the motors
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSecondary  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  

  // create a speed controller group for each side
  //private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
  //private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

  // create a drive train group with the speed controller groups
  //private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);

  //navx
  // private AHRS navx = new AHRS();

  public SUB_Drivetrain() {
    
    leftPrimary.setInverted(Constants.Drivetrain.kFrontLeftInverted);
    leftPrimary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    leftPrimary.setIdleMode(IdleMode.kCoast);
    leftPrimary.burnFlash();
  
    
    rightPrimary.setInverted(Constants.Drivetrain.kFrontRightInverted);
    rightPrimary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    rightPrimary.setIdleMode(IdleMode.kCoast);
    rightPrimary.burnFlash();
  
      
      leftSecondary.setInverted(Constants.Drivetrain.kRearLeftInverted);
      leftSecondary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
      leftSecondary.setIdleMode(IdleMode.kCoast);
      leftSecondary.burnFlash();
  
      
      rightSecondary.setInverted(Constants.Drivetrain.kRearRightInverted);
      rightSecondary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
      rightSecondary.setIdleMode(IdleMode.kCoast);
      rightSecondary.burnFlash();
    
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
  public void driveArcade(double _straight, double _turn) {
    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);


    leftPrimary.set(left);
    rightPrimary.set(right);
    leftSecondary.set(left);
    rightSecondary.set(right);

  }

  public void setMotorsArcade(double forwardSpeed, int turnSpeed) {
    //driveTrain.arcadeDrive(forwardSpeed, turnSpeed);
  }

  public void setMotorsTank(double leftSpeed, double rightSpeed) {
    //driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void setMotorsCurvature(double xSpeed, double zRotation, boolean isQuickTurn){
    //driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /* Encoders getting position
  public double getLeftEncoder(){
    return leftPrimary.getEncoder().getPosition();
  }

  public double getRightEncoder(){
    return rightPrimary.getEncoder().getPosition();
  }
*/
  

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
