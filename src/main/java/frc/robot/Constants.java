// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
  }
  public static final int ID_LEFT_PRIMARY = 20;
  public static final int ID_LEFT_SECONDARY = 22;
  public static final int ID_RIGHT_PRIMARY = 23;
  public static final int ID_RIGHT_SECONDARY = 25;

  public static final int JoystickPort = 0;
  
  public static double xDistance = 0;
  public static double yDistance = 0;
  public static double offset = 0;
  public static double distancedistance = 0;
}
