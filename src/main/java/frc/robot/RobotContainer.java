// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  //private XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController); 
  private Joystick controller = new Joystick(0);

  JoystickButton c_aButton = new JoystickButton(controller, 1);
  JoystickButton c_bButton = new JoystickButton(controller, 2);
  JoystickButton c_xButton = new JoystickButton(controller, 4);
  JoystickButton c_yButton = new JoystickButton(controller, 3);
  JoystickButton c_lBumper = new JoystickButton(controller, 5);
  JoystickButton c_rBumper = new JoystickButton(controller, 6);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //set up the drivetrain command that runs all the time
    m_drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        m_drivetrain.driveArcade(
          MathUtil.applyDeadband(- controller.getRawAxis(1), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(controller.getRawAxis(4)*Constants.Drivetrain.kTurningScale, Constants.OIConstants.kDriveDeadband))
  , m_drivetrain)
    );
     
    //set up gripper open/close
    c_rBumper
    .onTrue(new InstantCommand(() -> m_gripper.openGripper()))
    .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));

    //set up arm preset positions
    c_aButton
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
    c_bButton      
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition, m_gripper)));
   c_yButton
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition, m_gripper)));
    c_xButton
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition, m_gripper)));

    //set up arm manual and auto functions
    m_arm.setDefaultCommand(new RunCommand(
      () ->
        m_arm.runAutomatic()
      , m_arm)
    );
    new Trigger(() -> 
      Math.abs(controller.getRawAxis(3) - controller.getRawAxis(2)) > Constants.OIConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
          m_arm.runManual((controller.getRawAxis(3) - controller.getRawAxis(2
          )) * Constants.OIConstants.kArmManualScale)
        , m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
