// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.util.sendable.Sendable;


public class SUB_Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  // Gets the motors
  private CANSparkMax leftPrimary = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftSecondary = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightPrimary = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightSecondary  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder leftPrimaryEncoder = leftPrimary.getEncoder();
  private RelativeEncoder rightPrimaryEncoder = rightPrimary.getEncoder();
  private RelativeEncoder leftSecondaryEncoder = leftSecondary.getEncoder();
  private RelativeEncoder rightSecondaryEncoder = rightSecondary.getEncoder();  

   // The gyro sensor
   private AHRS navx = new AHRS(SerialPort.Port.kMXP);

   //Field Map
   private Field2d field2d;

   // Odometry class for tracking robot pose
   DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()), this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
   new Pose2d(0, 0, new Rotation2d()));

  // create a speed controller group for each side
  //private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
  //private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

  // create a drive train group with the speed controller groups
  //private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);

  //navx
  // private AHRS navx = new AHRS();

  public SUB_Drivetrain(Field2d input) {
    this.field2d = input;
    navx.setAngleAdjustment(0.0);
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

   _straight=  Math.copySign(Math.pow(_straight, 2), _straight);
   _turn=  Math.copySign(Math.pow(_turn, 2), _turn);


    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);


    leftPrimary.set(left);
    rightPrimary.set(right);
    leftSecondary.set(left);
    rightSecondary.set(right);

  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftPrimary.setVoltage(leftVolts);
    leftSecondary.setVoltage(leftVolts);
    rightPrimary.setVoltage(rightVolts);
    rightSecondary.setVoltage(rightVolts);

  public void driveArcadeSquared(double _straight, double _turn) {
    driveArcade(Math.pow(_straight,2), Math.pow(_turn,2));


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

 /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
   * @param input rpm of drivetrain motor
   * @return returns rate of encoder in meters per second
   */
  public double getRate(double input) {
    return  (input / Constants.Autonomous.GEARRATIO) * ((2 * Math.PI * Units.inchesToMeters(Constants.Autonomous.WHEEL_RADIUS)) / 60);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getRate(leftPrimaryEncoder.getVelocity()), getRate(rightPrimaryEncoder.getVelocity()));
  }

  
  /**
   * 
   * @param input encoder rotations from sparkmax
   * @return meters the robot has moved
   */
  public double rotationsToMeters(double input) {
    double wheelCirc = (2 * Math.PI * Constants.Autonomous.WHEEL_RADIUS);
    double rotationsPerInch = wheelCirc / Constants.Autonomous.GEARRATIO;
    return Units.inchesToMeters(rotationsPerInch * input);

  }

  /**
   * sets what the motor does while idle
   * 
   * @param input the mode the moros should be put in (kBrake or kCoast)
   */
  public void setIdleMode(IdleMode input) {
    leftPrimary.setIdleMode(input);
    leftSecondary.setIdleMode(input);
    rightPrimary.setIdleMode(input);
    rightSecondary.setIdleMode(input);
  }

  /**
   * 
   * @return rotation2d object with current heading
   */
  public Rotation2d getGyroHeading() {
    
    return new Rotation2d(Math.toRadians(-1 * navx.getYaw()));
  }

   /**
   * Sets the robot's current pose to the given x/y/angle.
   * 
   * @param x     The x coordinate
   * @param y     The y coordinate
   * @param angle The rotation component
   */
  public void setPosition(double x, double y, Rotation2d angle) {
    setPosition(new Pose2d(x, y, angle));
    navx.setAngleAdjustment(angle.getDegrees());
    zeroEncoders();
  }

  /**
   * Sets the robot's current pose to the given Pose2d.
   * 
   * @param position The position (both translation and rotation)
   */
  public void setPosition(Pose2d position) {
    driveOdometry.resetPosition(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()), this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
    new Pose2d(0, 0, new Rotation2d()));
    zeroEncoders();

  }

  /**
   * sets current heading to zero
   */
  public void zeroHeading() {
    navx.zeroYaw();
  }

  /**
   * zeros the encoder rotations
   */
  public void zeroEncoders() {
    leftPrimaryEncoder.setPosition(0);
    rightPrimaryEncoder.setPosition(0);
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Primary Encoder", leftPrimaryEncoder.getPosition());
    SmartDashboard.putNumber("Left Secondary Encoder", leftSecondaryEncoder.getPosition());
    SmartDashboard.putNumber("Right Primary Encoder", rightPrimaryEncoder.getPosition());
    SmartDashboard.putNumber("Right Secondary Encoder", rightSecondaryEncoder.getPosition());

    driveOdometry.update(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()),
    this.rotationsToMeters(rightPrimaryEncoder.getPosition()));

    field2d.setRobotPose(driveOdometry.getPoseMeters());

  }
}

