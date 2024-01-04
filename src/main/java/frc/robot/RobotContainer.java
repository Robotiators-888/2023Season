// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

 

  public static final Field2d field2d = new Field2d();
  //public static SendableChooser<Double> AutoBalanceStopAngleChooser = new SendableChooser<>();

  //public static final SUB_Gripper gripper = new SUB_Gripper();
  public static final SUB_Drivetrain drivetrain = new SUB_Drivetrain(field2d);


  private final static Joystick controller = new Joystick(Constants.JOYSTICK_PORT);
  //private final static Joystick controller2 = new Joystick(Constants.JOYSTICK_PORT2);

 // Auto objects
 public static SendableChooser<Command> AutoChooser = new SendableChooser<>();
 SendableChooser<Integer> DelayChooser = new SendableChooser<>();

 /**
   * The state of the buttons on the joystick.
   *
   * @param stick The joystick to read.
   * @return The state of the buttons on the joystick.
   */
  // public static int getStickButtons(final int stick) {
  //   if (stick < 0 || stick >= 2) {
  //     throw new IllegalArgumentException("Joystick index is out of range, should be 0-3");
  //   }

  //  // return (int) logDS.getJoystickData(stick).buttonValues;
  // }

 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   

  
    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        drivetrain.driveArcade(
          MathUtil.applyDeadband(- controller.getRawAxis(1), Constants.OperatorConstants.kDriveDeadband),
          MathUtil.applyDeadband(controller.getRawAxis(4)*Constants.Drivetrain.kTurningScale, Constants.OperatorConstants.kDriveDeadband))
  , drivetrain)
    );

    
  }
  

  public static void robotPeriodic() {
    logDriverData();
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command chosenAuto = AutoChooser.getSelected();
    int delay = DelayChooser.getSelected();
    return new SequentialCommandGroup(new WaitCommand(delay), chosenAuto);
  }


   



public static void logDriverController() {
  Logger.getInstance().recordOutput("Driver1Controller/leftAxis", controller.getRawAxis(Constants.LEFT_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/RightYAxis", controller.getRawAxis(Constants.RIGHT_Y_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/RightXAxis", controller.getRawAxis(Constants.RIGHT_X_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/BButton", controller.getRawButtonPressed(2));

}


public static void logDriverData(){
  logDriverController();
}

}