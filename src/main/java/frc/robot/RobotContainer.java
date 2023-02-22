// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.AprilTag.CMD_AprilSequential;
import frc.robot.commands.Limelight.CMD_LimeSequential;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick controller = new Joystick(Constants.JoystickPort);
  private Joystick controller2 = new Joystick(Constants.JoystickPort2);
  private SUB_Limelight limelight = new SUB_Limelight();
  private SUB_Drivetrain drivetrain = new SUB_Drivetrain();
  private SUB_AprilTag apriltag = new SUB_AprilTag();
  private final SUB_Gripper gripper = new SUB_Gripper();
  private final SUB_Tower tower = new SUB_Tower();
  private CMD_LimeSequential LimeSequential = new CMD_LimeSequential(drivetrain, limelight,gripper,tower);
  private CMD_AprilSequential AprilSequential = new CMD_AprilSequential(drivetrain, apriltag,gripper,tower);
  // The robot's subsystems and commands are defined here...
  private JoystickButton c_rBumper = new JoystickButton(controller, 5);
  private JoystickButton c_lBumper = new JoystickButton(controller, 6);
  private JoystickButton c_aButton = new JoystickButton(controller, 1);
  private JoystickButton c_bButton = new JoystickButton(controller, 2);
  private JoystickButton c_yButton = new JoystickButton(controller, 3);
  private JoystickButton c_xButton = new JoystickButton(controller, 4);

  JoystickButton c2_yButton = new JoystickButton(controller2, 4);
  JoystickButton c2_bButton = new JoystickButton(controller2, 2);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0)
    .setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 416, 240, 180));
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
    c2_yButton.toggleOnTrue(LimeSequential);
    c2_bButton.toggleOnTrue(AprilSequential);
    
    c_lBumper
    .onTrue(new InstantCommand(() -> {gripper.openGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}))
    .onFalse(new InstantCommand(() -> {gripper.closeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}));
    //.onFalse(new InstantCommand(() -> {m_gripper.driveGripper(-0.25);SmartDashboard.putNumber("Gripper Status", m_gripper.getSetPosition());}));
    
    /* 
    c_rBumper
    .onTrue(new RunCommand(()-> {m_gripper.driveGripper(0.25);}, m_gripper))
    .onFalse(new RunCommand(()->{m_gripper.driveGripper(0.0);}, m_gripper));
    */
    c_rBumper
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