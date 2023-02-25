// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Tower;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Gripper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.blinkin;

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
  public final static blinkin m_blinkin = new blinkin(Constants.KBLINKIN);

  JoystickButton rBumper = new JoystickButton(controller, 5);
  JoystickButton lBumper = new JoystickButton(controller, 6);

  //TODO: Adjust buttons and button numbers as needed
  final JoystickButton abutton = new JoystickButton(controller, 1);
  final JoystickButton bbutton = new JoystickButton(controller, 2);
  final JoystickButton xbutton = new JoystickButton(controller, 3);
  final JoystickButton ybutton = new JoystickButton(controller, 4);

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

    //drivetrain.setDefaultCommand(new RunCommand( ()-> drivetrain.setMotorsArcade(controller.getRawAxis(Constants.LEFT_AXIS), 
    //controller.getRawAxis(Constants.RIGHT_X_AXIS)*Constants.TURNING_SCALE), drivetrain));

    // Curvature Drive
   // m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.setMotorsCurvature(controller.getRawAxis(Constants.LEFT_AXIS), 
    //    controller.getRawAxis(Constants.RIGHT_X_AXIS), controller.getRawButton(Constants.LEFT_TRIGGER)), m_drivetrain));

    //gripper.setDefaultCommand(new RunCommand(() -> {gripper.setMotors(0);},gripper));

   //While held this will open the gripper using a run command that executes the mehtod manually
   //lBumper.whileTrue(new RunCommand(() -> {gripper.setMotors(-0.1);}, gripper));

   //While held this will close the gripper using a run command that executes the mehtod manually
   //rBumper.whileTrue(new RunCommand(() -> {gripper.setMotors(0.1);}, gripper));

   defaultAllianceColor();
  
   // abutton.whileHeld(() -> m_addressableLED.rainbow(), m_addressableLED);
   //abutton.onTrue(new RunCommand(() -> {m_blinkin.set(0.65);}, m_blinkin)); //Orange
   //bbutton.onTrue(new RunCommand(()->{m_blinkin.set(-0.99);}, m_blinkin));  //Rainbow
   //xbutton.onTrue(new RunCommand(() -> {m_blinkin.set(-0.35);}, m_blinkin)); //Red Scanner
   //ybutton.onTrue(new RunCommand(() -> {m_blinkin.set(0.93);}, m_blinkin)); //White  
   
   abutton.onTrue(m_blinkin.solidRedCommand());
   bbutton.onTrue(m_blinkin.solidVioletCommand());
   xbutton.onTrue(m_blinkin.solidOrangeCommand());
   ybutton.onTrue(m_blinkin.allianceColorCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new CMD_AutoDrive(drivetrain).withTimeout(Constants.AUTO_TIME_SECS);
  }

public double defaultAllianceColor(){
  boolean isRed = (DriverStation.getAlliance() == Alliance.Red);
  if (isRed){
    
    return-0.35;
    
  } else {
    return 0.87;
    
  }
}

}