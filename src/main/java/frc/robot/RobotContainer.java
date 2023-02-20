// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.AprilTag.CMD_AprilSequential;
import frc.robot.commands.Limelight.CMD_LimeSequential;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick controller2 = new Joystick(Constants.JoystickPort);
  private SUB_Limelight limelight = new SUB_Limelight();
  private SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  private SUB_AprilTag apriltag = new SUB_AprilTag();
  private CMD_LimeSequential LimeSequential = new CMD_LimeSequential(drivetrain, limelight);
  private CMD_AprilSequential AprilSequential = new CMD_AprilSequential(drivetrain, apriltag);

  
  JoystickButton yButton = new JoystickButton(controller2, 4);
  JoystickButton bButton = new JoystickButton(controller2, 2);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    configureBindings();
    limelight.setLed(3);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Press the Y button once, then we will start the sequence and press it again we stop
    yButton.toggleOnTrue(LimeSequential);
    bButton.toggleOnTrue(AprilSequential);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The auto code
    return LimeSequential;
  }
}
