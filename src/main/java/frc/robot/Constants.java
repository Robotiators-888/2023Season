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

  public static final int ID_LEFT_PRIMARY = 20;
  public static final int ID_LEFT_SECONDARY = 22;
  public static final int ID_RIGHT_PRIMARY = 23;
  public static final int ID_RIGHT_SECONDARY = 25;

  public static final int JOYSTICK_PORT = 0;

  public static final int LEFT_AXIS = 1;
  public static final int RIGHT_AXIS = 5;
  public static final int LEFT_TRIGGER = 2;
  public static final double DEAD_ZONE = 0.3;

  public static final double WHEEL_RADIUS = 3; // wheel radius in inches
  public static final double GEARRATIO = 10.86; //gear ratio from output shaft of motor to wheel axle
 
  public static final int DRIVER_CONTROLLER = 0;

  public static final double AUTO_SPEED = 0.2; 
  public static final double AUTO_TIME_SECS = 5;

  public static final double TELESPEED = 0.45;
}
