// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

//Drivetrain CANIDs
  public static final int ID_LEFT_PRIMARY = 20;
  public static final int ID_LEFT_SECONDARY = 22;
  public static final int ID_RIGHT_PRIMARY = 23;
  public static final int ID_RIGHT_SECONDARY = 25;

//Joystick Constants
  public static final int JOYSTICK_PORT = 0;
  public static final int DRIVER_CONTROLLER = 0;

  public static final int LEFT_AXIS = 1;
  public static final int RIGHT_X_AXIS = 4;
  public static final int RIGHT_Y_AXIS = 5;
  public static final int LEFT_TRIGGER = 2;

  public static final double DEAD_ZONE = 0.3;
  public static final double TURNING_SCALE = 0.5;

  public static final double WHEEL_RADIUS = 3; // wheel radius in inches
  public static final double GEARRATIO = 10.86; //gear ratio from output shaft of motor to wheel axle

  public static final double AUTO_SPEED = 0.45; 
  public static final double AUTO_TIME_SECS = 15;

  public static final double TELESPEED = 0.45;

//Manipulator CANIDS
  public static final int TOWER_SPARKMAX_CAN_ID = 10;

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
  
//Gripper CANIDS
  public static final int GRIPPER_SPARKMAX_CAN_ID = 11;
  

}
