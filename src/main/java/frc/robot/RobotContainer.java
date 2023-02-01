// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DataLogManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick controller = new Joystick(Constants.JoystickPort);
  private SUB_Limelight limelight = new SUB_Limelight();
  private SUB_Drivetrain drivetrain = new SUB_Drivetrain();

  
  JoystickButton C_aButton = new JoystickButton(controller, 1);


  // The robot's subsystems and commands are defined here...
  private final SUB_Gripper gripper = new SUB_Gripper();
  private final SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  private final SUB_Tower tower = new SUB_Tower();
  private final Joystick controller = new Joystick(Constants.JOYSTICK_PORT);

  private JoystickButton c_rBumper = new JoystickButton(controller, 5);
  private JoystickButton c_lBumper = new JoystickButton(controller, 6);
  private JoystickButton c_aButton = new JoystickButton(controller, 1);
  private JoystickButton c_bButton = new JoystickButton(controller, 2);
  private JoystickButton c_yButton = new JoystickButton(controller, 3);
  private JoystickButton c_xButton = new JoystickButton(controller, 4);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0)
    .setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 416, 240, 180));
    // Configure the trigger bindings
    
    configureBindings();
    limelight.setLed(1);
    C_aButton.whileTrue(new CMD_LimeSequential(drivetrain, limelight));
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
     limelight.setLed(0);

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
        drivetrain.setDefaultCommand(new CMD_TeleDrive(drivetrain, () -> -controller.getRawAxis(1),
                                () -> -controller.getRawAxis(4)));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


   //While held this will open the gripper using a run command that executes the mehtod manually


    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        drivetrain.driveArcade(
          MathUtil.applyDeadband(- controller.getRawAxis(1), Constants.OperatorConstants.kDriveDeadband),
          MathUtil.applyDeadband(controller.getRawAxis(4)*Constants.Drivetrain.kTurningScale, Constants.OperatorConstants.kDriveDeadband))
  , drivetrain)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new CMD_LimeSequential(drivetrain, limelight);
  }
}