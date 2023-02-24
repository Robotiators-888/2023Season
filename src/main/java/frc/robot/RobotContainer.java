// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...

  public static final Field2d field2d = new Field2d();

  public static final SUB_Gripper gripper = new SUB_Gripper();
  public static final SUB_Drivetrain drivetrain = new SUB_Drivetrain(field2d);
  public static final SUB_Tower tower = new SUB_Tower();
  private static final Autonomous autos = new Autonomous();
  
  private final Joystick controller = new Joystick(Constants.JOYSTICK_PORT);
  private final Joystick controller2 = new Joystick(Constants.JOYSTICK_PORT2);
  private JoystickButton c_rBumper = new JoystickButton(controller2, 5);
  private JoystickButton c_lBumper = new JoystickButton(controller2, 6);
  private JoystickButton c_aButton = new JoystickButton(controller2, 1);
  private JoystickButton c_bButton = new JoystickButton(controller2, 2);
  private JoystickButton c_yButton = new JoystickButton(controller2, 4);
  private JoystickButton c_xButton = new JoystickButton(controller2, 3);



 // Auto objects
 SendableChooser<Command> AutoChooser = new SendableChooser<>();
 SendableChooser<Integer> DelayChooser = new SendableChooser<>();

 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0)
    .setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 416, 240, 180));
    // Configure the trigger bindings
    AutoChooser.setDefaultOption("Red 1 - One Game Piece", autos.red1_1GP);

    DelayChooser.setDefaultOption("0 sec", 0);
    DelayChooser.addOption("1 sec", 1);
    DelayChooser.addOption("2 sec", 2);
    DelayChooser.addOption("3 sec", 3);
    DelayChooser.addOption("4 sec", 4);
    DelayChooser.addOption("5 sec", 5);
    DelayChooser.addOption("6 sec", 6);
    DelayChooser.addOption("7 sec", 7);
    DelayChooser.addOption("8 sec", 8);
    DelayChooser.addOption("9 sec", 9);
    DelayChooser.addOption("10 sec", 10);

    SmartDashboard.putData("Auto Chooser", AutoChooser);
    SmartDashboard.putData("Delay Chooser", DelayChooser);



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
    c_lBumper
    .onTrue(new InstantCommand(() -> {gripper.openConeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}))
    .onFalse(new InstantCommand(() -> {gripper.closeConeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}));
    //.onFalse(new InstantCommand(() -> {m_gripper.driveGripper(-0.25);SmartDashboard.putNumber("Gripper Status", m_gripper.getSetPosition());}));
    
    /* 
    c_rBumper
    .onTrue(new RunCommand(()-> {gripper.openCubeGripper();}, gripper))
    .onFalse(new RunCommand(()->{gripper.closeCubeGripper();}, gripper));
    */

    // default case, balances arm without changing position.
    tower.setDefaultCommand(new RunCommand(() -> {tower.armMoveVoltage(0);},tower));
    // buttons, move arm forward and backward
    //set up arm preset positions
    c_aButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)));
    c_bButton      
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)));
   c_yButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kIntakePosition, tower)));
    c_xButton
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kFeederPosition, tower)),
         new SequentialCommandGroup(
          new WaitCommand(0.25), 
          new InstantCommand(() -> gripper.openConeGripper(), gripper))));
    //Creates a default command for runing the tower down using the right trigger
    tower.setDefaultCommand(new RunCommand(
      () ->
      tower.runAutomatic()
      , tower)
    );
    new Trigger(() -> 
      Math.abs(Math.pow(controller2.getRawAxis(3), 2) - Math.pow(controller2.getRawAxis(2), 2)) > Constants.OperatorConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
        tower.runManual((Math.pow(controller2.getRawAxis(3), 2) - Math.pow(controller2.getRawAxis(2), 2)) * Constants.OperatorConstants.kArmManualScale)
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
    Command chosenAuto = AutoChooser.getSelected();
    int delay = DelayChooser.getSelected();
    drivetrain.zeroEncoders();
    drivetrain.zeroHeading();
    return new SequentialCommandGroup(new WaitCommand(delay), chosenAuto);
}
}