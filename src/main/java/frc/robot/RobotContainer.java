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
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
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

  public static final SUB_Gripper gripper = new SUB_Gripper();
  public static final SUB_Drivetrain drivetrain = new SUB_Drivetrain(field2d);
  public static final SUB_Tower tower = new SUB_Tower();
  public static SUB_Limelight limelight = new SUB_Limelight();
  public static SUB_AprilTag apriltag = new SUB_AprilTag();
  public static CMD_LimeSequential LimeSequential = new CMD_LimeSequential();
  public static CMD_AprilSequential AprilSequential = new CMD_AprilSequential();
  public static final Autonomous autos = new Autonomous();
  private static LoggedDriverStation logDS = LoggedDriverStation.getInstance();
  public final static SUB_Blinkin m_blinkin = new SUB_Blinkin(Constants.KBLINKIN);

  
  private final static Joystick controller = new Joystick(Constants.JOYSTICK_PORT);
  private final static Joystick controller2 = new Joystick(Constants.JOYSTICK_PORT2);
  

  private JoystickButton d_rBumper = new JoystickButton(controller, 5);
  private JoystickButton d_aButton = new JoystickButton(controller, 1);
  private JoystickButton d_bButton = new JoystickButton(controller, 2);

  private JoystickButton c_rBumper = new JoystickButton(controller2, 5);
  private JoystickButton c_lBumper = new JoystickButton(controller2, 6);
  private JoystickButton c_aButton = new JoystickButton(controller2, 1);
  private JoystickButton c_bButton = new JoystickButton(controller2, 2);
  private JoystickButton c_yButton = new JoystickButton(controller2, 4);
  private JoystickButton c_xButton = new JoystickButton(controller2, 3);

  JoystickButton c0_yButton = new JoystickButton(controller, 4);
  JoystickButton c0_bButton = new JoystickButton(controller, 2);
  JoystickButton c0_xButton = new JoystickButton(controller, 3);
  JoystickButton c0_aButton = new JoystickButton(controller, 1);

 // Auto objects
 public static SendableChooser<Command> AutoChooser = new SendableChooser<>();
 SendableChooser<Integer> DelayChooser = new SendableChooser<>();

 /**
   * The state of the buttons on the joystick.
   *
   * @param stick The joystick to read.
   * @return The state of the buttons on the joystick.
   */
  public static int getStickButtons(final int stick) {
    if (stick < 0 || stick >= 2) {
      throw new IllegalArgumentException("Joystick index is out of range, should be 0-3");
    }

    return (int) logDS.getJoystickData(stick).buttonValues;
  }

 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   

    CameraServer.startAutomaticCapture()
    .setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 416, 240, 60));

    AutoChooser.setDefaultOption("Place 1 Cone", autos.buildScoringSequence());
    AutoChooser.addOption("Red 1 - One Cone DriveBack", autos.Red1_Cone_DB());
    AutoChooser.addOption("Red 3 - One Cone DriveBack", autos.Red3_Cone_DB());
    AutoChooser.addOption("Blue 1 - One Cone DriveBack", autos.Blue1_Cone_DB());
    AutoChooser.addOption("Blue 3 - One Cone DriveBack", autos.Blue3_Cone_DB());
    AutoChooser.addOption("Game Piece", autos.DriveToGamePiece());
    AutoChooser.addOption("Curvy Drive To GP", autos.Curvy_DTP());

   // AutoChooser.addOption("Auto Balance Only", autos.autoBalanceSequence);
    AutoChooser.addOption("1 Cone Auto Balance", autos.Cone_AutoBalance());
    AutoChooser.addOption("score Then AutoBalance Backwards", autos.backwardsScoreThenAutoBalance());
    AutoChooser.addOption("Test Auto Balance", autos.buildAutoBalanceSequence()); 
    AutoChooser.addOption("Test Turn 180", autos.turn180Degree());




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
    //SmartDashboard.putData("AutoBalanceStopAngleChooser",AutoBalanceStopAngleChooser);



    configureBindings();
    limelight.setLed(1);

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
    limelight.setDefaultCommand(new InstantCommand(() -> limelight.setLed(1), limelight));
    // Press the Y button once, then we will start the sequence and press it again we stop
    // Press the B button once, then the april tag sequence will start
    c0_yButton.onTrue(LimeSequential);
    c0_bButton.onTrue(AprilSequential);
    
    
    c_lBumper
    .onTrue(new InstantCommand(() -> {gripper.openConeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}))
    .onFalse(new InstantCommand(() -> {gripper.closeConeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}));
    //.onFalse(new InstantCommand(() -> {m_gripper.driveGripper(-0.25);SmartDashboard.putNumber("Gripper Status", m_gripper.getSetPosition());}));
    
    /* 
    c_rBumper
    .onTrue(new RunCommand(()-> {gripper.openCubeGripper();}, gripper))
    .onFalse(new RunCommand(()->{gripper.closeCubeGripper();}, gripper));
    */

    d_rBumper
    .onTrue(new InstantCommand(()->drivetrain.toggleBrake()));

   d_bButton
      .onTrue(new InstantCommand(() -> {gripper.openConeGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}));

    // default case, balances arm without changing position.
    tower.setDefaultCommand(new RunCommand(() -> {tower.armMoveVoltage(0);},tower));
    // buttons, move arm forward and backward
    //set up arm preset positions
    c_aButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)));
    c_bButton      
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)));
   c_yButton
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kIntakePosition, tower)),
         new SequentialCommandGroup(
          new WaitCommand(0.25), 
          new InstantCommand(() -> gripper.openCubeGripper(), gripper))));
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
  
 
  
    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        drivetrain.driveArcade(
          MathUtil.applyDeadband(- controller.getRawAxis(1), Constants.OperatorConstants.kDriveDeadband),
          MathUtil.applyDeadband(controller.getRawAxis(4)*Constants.Drivetrain.kTurningScale, Constants.OperatorConstants.kDriveDeadband))
  , drivetrain)
    );

    
  }

  public double defaultAllianceColor(){
    boolean isRed = (DriverStation.getAlliance() == Alliance.Red);
    if (isRed){
      
      return-0.35;
      
    } else {
      return 0.87;
      
    }
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

}

public static void logOperatorController() {
  Logger.getInstance().recordOutput("Driver2Controller/AButton", controller2.getRawButtonPressed(1));
  Logger.getInstance().recordOutput("Driver2Controller/BButton", controller2.getRawButtonPressed(2));
  Logger.getInstance().recordOutput("Driver2Controller/YButton", controller2.getRawButtonPressed(3));
  Logger.getInstance().recordOutput("Driver2Controller/XButton", controller2.getRawButtonPressed(4));
  Logger.getInstance().recordOutput("Driver2Controller/RightShoulderButton", controller2.getRawButtonPressed(6));
}

public static void logDriverData(){
  logDriverController();
  logOperatorController();

}

}