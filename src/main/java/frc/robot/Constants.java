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
  public static final int JOYSTICKPORT = 0;

  //Manip Sub
  public static final int TOWER_SPARKMAX_CAN_ID = 10;
  public static final int GRIPPER_SPARKMAX_CAN_ID = 11;

  //Arbitrary PID and FF values, will tune later
  public static final double PID_kP = 0.2;
  public static final int PID_kI = 0;
  public static final double PID_kD = 0.1;
  public static final double FF_kA = 0.0;
  public static final double FF_kG = 20.0;
  public static final double FF_kS = 19.0;
  public static final double FF_kV = 1.0; 
  public static final double FF_Velocity = 1.0;
  public static final double FF_Accel = 1.0;
  
}
