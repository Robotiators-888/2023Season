// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.function.Supplier;

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
  private boolean reversed = false;

   // The gyro sensor
   private AHRS navx = new AHRS(SerialPort.Port.kMXP);

   //Field Map
   private Field2d field2d;

   // Odometry class for tracking robot pose
   DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getGyroHeading(), leftPrimaryEncoder.getPosition(), 
   rightSecondaryEncoder.getPosition(),
   new Pose2d(0, 0, new Rotation2d()));

  // create a speed controller group for each side
  private MotorControllerGroup groupLeft = new MotorControllerGroup(leftPrimary, leftSecondary);
  private MotorControllerGroup groupRight = new MotorControllerGroup(rightPrimary, rightSecondary);

  // create a drive train group with the speed controller groups
  private DifferentialDrive driveTrain = new DifferentialDrive(groupLeft, groupRight);

  boolean brake = false;

  //navx
  // private AHRS navx = new AHRS();

  public SUB_Drivetrain(Field2d input) {
    this.field2d = input;
    zeroHeading();
    navx.setAngleAdjustment(0.0);

    leftPrimary.setInverted(Constants.Drivetrain.kFrontLeftInverted);
    leftPrimary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    leftPrimary.setIdleMode(IdleMode.kBrake);
    leftPrimary.burnFlash();
    //leftPrimaryEncoder.setInverted(true);
  
    
    rightPrimary.setInverted(Constants.Drivetrain.kFrontRightInverted);
    rightPrimary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    rightPrimary.setIdleMode(IdleMode.kBrake);
    rightPrimary.burnFlash();
  
      
      leftSecondary.setInverted(Constants.Drivetrain.kRearLeftInverted);
      leftSecondary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
      leftSecondary.setIdleMode(IdleMode.kBrake);
      leftSecondary.burnFlash();
      //rightPrimaryEncoder.setInverted(true);
      
      rightSecondary.setInverted(Constants.Drivetrain.kRearRightInverted);
      rightSecondary.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
      rightSecondary.setIdleMode(IdleMode.kBrake);
      rightSecondary.burnFlash();

      setBrakeMode(true);
      
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

public double invertEncoderVal(double currentVal){
  return -currentVal;
}

  /**
   * Sets the reverse variable ito given value
   * @param state value to set reversed to
   */
  public void setReverse(boolean state){
    reversed = state;

    if(reversed){
      leftPrimary.setInverted(!reversed);
      rightPrimary.setInverted(reversed);
      leftSecondary.setInverted(!reversed);
      rightSecondary.setInverted(reversed);
      reversed = false;
    }else {
      leftPrimary.setInverted(!reversed);
      rightPrimary.setInverted(reversed);
      leftSecondary.setInverted(!reversed);
      rightSecondary.setInverted(reversed);
      reversed = true;
    }

  }

  /**
   * toggles state of reverse variable
   */
  public void toggleReverse(){
    if(reversed){
      leftPrimary.setInverted(!reversed);
      rightPrimary.setInverted(reversed);
      leftSecondary.setInverted(!reversed);
      rightSecondary.setInverted(reversed);
      reversed = false;
    }else {
      leftPrimary.setInverted(!reversed);
      rightPrimary.setInverted(reversed);
      leftSecondary.setInverted(!reversed);
      rightSecondary.setInverted(reversed);
      reversed = true;
    }
  }

  /**
   * @return boolean sotred in reverse variable
   */
  public boolean getReverse(){
    return reversed;
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
    driveTrain.feedWatchdog();

  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    groupLeft.setVoltage(rightVolts);
    groupRight.setVoltage(leftVolts);
    driveTrain.feedWatchdog();

  }

  
  public void driveArcadeSquared(double _straight, double _turn) {
    driveArcade(_straight,_turn);
    driveTrain.feedWatchdog();
  }

  public void setMotorsArcade(double forwardSpeed, int turnSpeed) {
    //driveTrain.arcadeDrive(forwardSpeed, turnSpeed);
  }

  public void setMotorsTank(Supplier<Double> lSpeed, Supplier<Double> rSpeed) {
    
    double leftSpeed = Math.copySign(Math.pow(lSpeed.get(), 2), lSpeed.get());
    double rightSpeed = Math.copySign(Math.pow(rSpeed.get(), 2), rSpeed.get());
    
    groupLeft.set(leftSpeed);
    groupRight.set(rightSpeed);
    driveTrain.feedWatchdog();
  }

  public void setMotorsTank(double leftSpeed, double rightSpeed){

     leftSpeed = Math.copySign(Math.pow(leftSpeed, 2), leftSpeed);
     rightSpeed = Math.copySign(Math.pow(rightSpeed, 2), rightSpeed);

     driveTrain.tankDrive(leftSpeed, rightSpeed);
     driveTrain.feedWatchdog();
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
    
    return new Rotation2d(-1*Math.toRadians(navx.getYaw()));
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
    //driveOdometry.resetPosition(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()), this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
    //new Pose2d(0, 0, new Rotation2d()));
    zeroEncoders();
    driveOdometry.resetPosition(navx.getRotation2d(), leftPrimaryEncoder.getPosition(), rightPrimaryEncoder.getPosition(), position);

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


   public double getAngle(){
     return navx.getAngle();
   }

   public void resetAngle(){
     navx.reset();
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

  // Gets the number from the smart dashboard to change drive
  // public int driveMode(){
  //   return (int) SmartDashboard.getNumber("Drive Mode", 0);
  // }

  // Switches it?

  public void turn180Degree(){
    double degree = getYaw();
    //posative turn is left
    if (degree < 180) { // turn left
      double turnSpeed = -Math.min(Math.max(degree * -0.03, -0.3),-0.265);
      this.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
      SmartDashboard.putNumber("Turn180 TurnSpeed: ", turnSpeed);
    } else if (degree > 180){ // turn right
        double turnSpeed = -Math.max(Math.min(degree * -0.03, 0.3),0.265);
        this.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
        SmartDashboard.putNumber("Turn180 TurnSpeed: ", turnSpeed);
    }
  }

  public void toggleBrake(){
    if(brake){
      brake = false;
    }else{
      brake = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Field",field2d);
    SmartDashboard.putBoolean("BREAK MODE", brake);
    SmartDashboard.putNumber("Left Primary Encoder", leftPrimaryEncoder.getPosition());
    SmartDashboard.putNumber("Left Secondary Encoder", leftSecondaryEncoder.getPosition());
    SmartDashboard.putNumber("Right Primary Encoder", rightPrimaryEncoder.getPosition());
    SmartDashboard.putNumber("Right Secondary Encoder", rightSecondaryEncoder.getPosition());
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Pose X", driveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", driveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Theta", driveOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Heading", getGyroHeading().getDegrees());


    driveOdometry.update(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()),
    this.rotationsToMeters(rightPrimaryEncoder.getPosition()));

    field2d.setRobotPose(driveOdometry.getPoseMeters());
    setBrakeMode(brake);

  }


  public double countEncoder(){
    
    
    return 0.0;
  }
}

