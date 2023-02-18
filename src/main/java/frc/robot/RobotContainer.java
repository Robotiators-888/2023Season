// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DataLogManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final SUB_Gripper gripper = new SUB_Gripper();
  private final SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  private final SUB_Tower tower = new SUB_Tower();
  private Joystick controller = new Joystick(Constants.JOYSTICK_PORT);

  JoystickButton c_rBumper = new JoystickButton(controller, 5);
  JoystickButton c_lBumper = new JoystickButton(controller, 6);
  JoystickButton c_aButton = new JoystickButton(controller, 1);
  JoystickButton c_bButton = new JoystickButton(controller, 2);
  JoystickButton c_yButton = new JoystickButton(controller, 3);
  JoystickButton c_xButton = new JoystickButton(controller, 4);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings


    configureBindings();
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



    // Curvature Drive
   // m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.setMotorsCurvature(controller.getRawAxis(Constants.LEFT_AXIS), 
    //    controller.getRawAxis(Constants.RIGHT_X_AXIS), controller.getRawButton(Constants.LEFT_TRIGGER)), m_drivetrain));

    //Creates a default command for runing the tower up using the left trigger
    c_rBumper
    .onTrue(new InstantCommand(() -> {gripper.openGripper();}))
    .onFalse(new InstantCommand(() -> {gripper.closeGripper();}));
    //.onFalse(new InstantCommand(() -> {m_gripper.driveGripper(-0.25);SmartDashboard.putNumber("Gripper Status", m_gripper.getSetPosition());}));
    
    /* 
    c_rBumper
    .onTrue(new RunCommand(()-> {m_gripper.driveGripper(0.25);}, m_gripper))
    .onFalse(new RunCommand(()->{m_gripper.driveGripper(0.0);}, m_gripper));
    */
    c_lBumper
    .onTrue(new RunCommand(()-> {gripper.driveGripper(-0.25);}, gripper))
    .onFalse(new RunCommand(()->{gripper.driveGripper(0.0);}, gripper));
    // default case, balances arm without changing position.
    tower.setDefaultCommand(new RunCommand(() -> {tower.armMoveVoltage(0);},tower));
    // buttons, move arm forward and backward
    //set up arm preset positions
    c_aButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kHomePosition, gripper)));
    c_bButton      
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, gripper)));
   c_yButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kIntakePosition, gripper)));
    c_xButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kFeederPosition, gripper)));
    //Creates a default command for runing the tower down using the right trigger
    tower.setDefaultCommand(new RunCommand(
      () ->
      tower.runAutomatic()
      , tower)
    );
    new Trigger(() -> 
      Math.abs(controller.getRawAxis(3) - controller.getRawAxis(2)) > Constants.OperatorConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
        tower.runManual((controller.getRawAxis(3) - controller.getRawAxis(2
          )) * Constants.OperatorConstants.kArmManualScale)
        , tower));

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
    return null;
  }
}