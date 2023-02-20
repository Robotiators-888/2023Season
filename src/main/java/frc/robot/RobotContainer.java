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
  private Joystick controller = new Joystick(Constants.JoystickPort);
  private SUB_Limelight limelight = new SUB_Limelight();
  private SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  private SUB_AprilTag apriltag = new SUB_AprilTag();
  private CMD_LimeSequential LimeSequential = new CMD_LimeSequential(drivetrain, limelight);
  private CMD_AprilSequential AprilSequential = new CMD_AprilSequential(drivetrain, apriltag);

  
  JoystickButton lTrigger = new JoystickButton(controller, 4);
  JoystickButton rTrigger = new JoystickButton(controller, 2);

  // The robot's subsystems and commands are defined here...

  public SUB_Tower tower = new SUB_Tower();

  JoystickButton rBumper = new JoystickButton(controller, 5);
  JoystickButton lBumper = new JoystickButton(controller, 6);
  JoystickButton xButton = new JoystickButton(controller, 3);


  // Replace with CommandPS4Controller or CommandJoystick if needed


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
    lTrigger.toggleOnTrue(LimeSequential);
    rTrigger.toggleOnTrue(AprilSequential);
    

    // this is all for the teleop drive
    drivetrain.setDefaultCommand(new CMD_TeleDrive(drivetrain, () -> -controller.getRawAxis(1),
                                () -> -controller.getRawAxis(4)));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    //Creates a default command for runing the tower up using the left trigger

    // default case, balances arm without changing position.
    tower.setDefaultCommand(new RunCommand(() -> {tower.armMoveVoltage(0);},tower));
    // buttons, move arm forward and backward
    lBumper.whileTrue(new RunCommand(() -> {tower.armMoveVoltage(-2);/*voltage added onto feedforward(arm balancer)*/ }, tower));
    rBumper.whileTrue(new RunCommand(() -> {tower.armMoveVoltage(2);}, tower));
    //resets arm encoder
    xButton.whileTrue(new RunCommand(() -> {tower.resetEncoder();}, tower));


    //Creates a default command for runing the tower down using the right trigger


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
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
