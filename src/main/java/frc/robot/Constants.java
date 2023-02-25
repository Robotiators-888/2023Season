// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.libs.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Autonomous{
    public static final double kmaxVelocity = 5.0;
    public static final double kmaxAcceleration = 2.0;
    
    public static final double TRACKWIDTH = 35.5; // This is in inches 
    //1.399in in width per wheel
    public static final double WHEEL_RADIUS = 3; // wheel radius in inches
    public static final double GEARRATIO = 10.71; //gear ratio from output shaft of motor to wheel axle
   

    public static final DifferentialDriveKinematics kDriveKinematics = 
         new DifferentialDriveKinematics(Units.inchesToMeters(TRACKWIDTH));

    public static final double ksVolts = 0.15437;
    public static final double kvVoltsSecondsPerMeter = 0.86449;
    public static final double kaVoltsSecondsSquaredPerMeter = 0.083814;
    public static final double kpDriverVelocity = 1.028;

    public static final double kMaxSpeedMetersPerSecond = 0.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double KP = 0.021; // was 0.015, was 0.021
    public static final double KD = 0.2; // was 0.2, was 0.2
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1;
  
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmManualDeadband = 0.05;
    public static final double kArmManualScale = 0.5;
  }

  public static final class Drivetrain {
    public static final int kFrontLeftCanId = 20;
    public static final int kFrontRightCanId = 23;
    public static final int kRearLeftCanId = 22;
    public static final int kRearRightCanId = 25;

    public static final boolean kFrontLeftInverted = true;
    public static final boolean kFrontRightInverted = false;
    public static final boolean kRearLeftInverted = true;
    public static final boolean kRearRightInverted = false;

    public static final int kCurrentLimit = 55;

    public static final double kTurningScale = -0.5;
  }

//Joystick Constants
 public static final int JOYSTICK_PORT = 0;
 public static final int JOYSTICK_PORT2 = 1;
 public static final int DRIVER_CONTROLLER = 0;

 public static final int LEFT_AXIS = 1;
 public static final int RIGHT_X_AXIS = 4;
 public static final int RIGHT_Y_AXIS = 5;
 public static final int LEFT_TRIGGER = 2;

 public static final double DEAD_ZONE = 0.3;
 public static final double TURNING_SCALE = 0.5;

 
 public static final double AUTO_SPEED = 0.45; 
 public static final double AUTO_TIME_SECS = 15;

 public static final double TELESPEED = 0.45;

//Manipulator CANIDS
 public static final int TOWER_SPARKMAX_CANID = 10;

 //Arbitrary PID and FF values, will tune later
 public static final double PID_kP = 0.227;//0.227 0-4v
  public static final int PID_kI = 0;
  public static final double PID_kD = 0.1;
  public static final double FF_kA = 0.0;
  public static final double FF_kG = .73; //amount of volts to Overcome gravity on the arm, was 1
  public static final double FF_kS = 19.0;
  public static final double FF_kV = 1.0; 
  public static final double FF_Velocity = 1.0;
  public static final double FF_Accel = 1.0;

  
  public static final class Gripper {
    public static final int kGripperCanId = 11;
    public static final double kSoftLimitReverse = -20.0;
    public static final double kSoftLimitForward = 22.0;
    public static final double kCloseConePosition = 0.0;
    public static final double kCloseCubePosition = 10.0;
    public static final double kConePosition = 19.0;
    public static final double kCubePosition = 21.0;
    public static final double kSafePosition = 27.0;
    public static final int kCurrentLimit = 10;
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
  }
  
    public static final class Arm {
    public static final int kArmCanId = 10;
    public static final boolean kArmInverted = false;
    public static final int kCurrentLimit = 40;

    public static final double kSoftLimitReverse = 0.0;
    public static final double kSoftLimitForward = 4.6;

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0) * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12/3.09577776, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = .2;
    //public static final double kScoringPosition = 2.95;
    public static final double kScoringPosition = 2.89;
    public static final double kIntakePosition = 4.32;
    public static final double kFeederPosition = 2.42;
}

}
