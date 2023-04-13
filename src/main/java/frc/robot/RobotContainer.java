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
  public static final SUB_Tower tower = new SUB_Tower();
  public static SUB_Limelight limelight = new SUB_Limelight();
  public static SUB_AprilTag apriltag = new SUB_AprilTag();
  public static final SUB_Roller roller = new SUB_Roller();
  public static final SUB_Ultrasonic ultrasonic = new SUB_Ultrasonic();
  private static LoggedDriverStation logDS = LoggedDriverStation.getInstance();
  public final static SUB_Blinkin blinkin = new SUB_Blinkin(Constants.KBLINKIN);
  public static StateManager stateManager = new StateManager();
  private static final Autonomous autos = new Autonomous();


  private final static Joystick controller = new Joystick(Constants.JOYSTICK_PORT);
  private final static Joystick controller2 = new Joystick(Constants.JOYSTICK_PORT2);

  private JoystickButton d_rBumper = new JoystickButton(controller, 5);
  private JoystickButton d_backButton = new JoystickButton(controller, 7);
  private JoystickButton d_aButton = new JoystickButton(controller, 1);
  private JoystickButton d_bButton = new JoystickButton(controller, 2);

  private JoystickButton c_rBumper = new JoystickButton(controller2, 6);
  private JoystickButton c_lBumper = new JoystickButton(controller2, 5);
  private JoystickButton c_aButton = new JoystickButton(controller2, 1);
  private JoystickButton c_bButton = new JoystickButton(controller2, 2);
  private JoystickButton c_yButton = new JoystickButton(controller2, 4);
  private JoystickButton c_xButton = new JoystickButton(controller2, 3);

 // Auto objects
 public static SendableChooser<Command> AutoChooser = new SendableChooser<>();
 SendableChooser<Integer> DelayChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   

    CameraServer.startAutomaticCapture()
    .setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 416, 240, 60));

  //   AutoChooser.setDefaultOption("Place 1 Cone", autos.buildScoringSequence());
  //   AutoChooser.addOption("Red 1 - One Cone DriveBack", autos.Red1_Cone_DB());
  //   AutoChooser.addOption("Red 3 - One Cone DriveBack", autos.Red3_Cone_DB());
  //   AutoChooser.addOption("Blue 1 - One Cone DriveBack", autos.Blue1_Cone_DB());
  //   AutoChooser.addOption("Blue 3 - One Cone DriveBack", autos.Blue3_Cone_DB());
  //   AutoChooser.addOption("Red 2 Cube Hold", autos.REDTwoPieceHOLD());
  //   //AutoChooser.addOption("Red 2 Cube SPIT", autos.REDTwoPieceSPIT());
  //   AutoChooser.addOption("Red 2 Cube Cable", autos.BLUETwoCubeCable());
  //   AutoChooser.addOption("Blue 2 Cube Hold", autos.BLUETwoPieceHOLD());
  //  // AutoChooser.addOption("Blue 2 Cube SPIT", autos.BLUETwoPieceSPIT());
  //   AutoChooser.addOption("Blue 2 Cube Cable", autos.BLUETwoCubeCable());
  //   AutoChooser.addOption("One Up and Over", autos.UpAndOver());
  //   AutoChooser.addOption("Two Up and Over", autos.TwoGPUpAndOver());
  //   AutoChooser.addOption("Cube Auto Balance", autos.Cube_AutoBalance());
  //   AutoChooser.addOption("Test Auto Balance", autos.buildAutoBalanceSequence()); 
  //   AutoChooser.addOption("Test Turn 180", autos.turn180Degree());
  //   AutoChooser.addOption("Test Turn Zero", autos.turnToZero());

      
      AutoChooser.addOption("Dummy Path 5.24m", autos.dummyCommand());
      AutoChooser.addOption("Place One Cone", autos.placeOneCone());
      AutoChooser.setDefaultOption("Place One Cube", autos.placeOneCube());



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
    DelayChooser.setDefaultOption("0 sec", 0);


    SmartDashboard.putData("Auto Chooser", AutoChooser);
    SmartDashboard.putData("Delay Chooser", DelayChooser);
    //SmartDashboard.putData("AutoBalanceStopAngleChooser",AutoBalanceStopAngleChooser);



    configureBindings();
    blinkin.allianceColor();
  

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
    // ur code sux #bad #atholtonbtr #staymad #dontgetmad #bye

    // Cube|Cone Setter
    c_lBumper
    .onTrue(new InstantCommand(() -> {stateManager.toggleGP();}, stateManager));

    limelight.setDefaultCommand(new InstantCommand(() -> limelight.setLed(1), limelight));
    // Press the Y button once, then we will start the sequence and press it again we stop
    // Press the B button once, then the april tag sequence will start
    
    d_aButton      
    .onTrue(autos.buildScoringSequence());

    d_bButton
    .toggleOnTrue(new InstantCommand(() -> {stateManager.outtakeRoller();}))
    .toggleOnFalse(new InstantCommand(()->stateManager.stopRoller()));
    
    c_rBumper
    .toggleOnTrue(new InstantCommand(()->stateManager.intakeRoller()))
    .toggleOnFalse(new InstantCommand(()->stateManager.stopRoller()));  

    d_backButton
    .onTrue(new InstantCommand(()->drivetrain.toggleBrake()));

  //  d_bButton
  //    // .onTrue(new InstantCommand(() -> {gripper.openGripper();SmartDashboard.putNumber("Gripper Status", gripper.getSetPosition());}));
  //    .onTrue(new InstantCommand(()->stateManager.intakeRoller()));

    // default case, balances arm without changing position.
    tower.setDefaultCommand(new RunCommand(() -> {tower.armMoveVoltage(0);},tower));
    // buttons, move arm forward and backward
    //set up arm preset positions
    c_aButton
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)));

    //Uses cubes or cones depending 
    c_bButton      
      .onTrue(new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)));
      
   c_yButton
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
         new SequentialCommandGroup(
          new WaitCommand(0.25))));
          //new InstantCommand(() -> gripper.openGripper(), gripper))));
    c_xButton
      .onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(stateManager.kFeederPosition(), tower)),
         new SequentialCommandGroup(
          new WaitCommand(0.25))));
          //new InstantCommand(() -> gripper.openGripper(), gripper))));

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
    return new SequentialCommandGroup(new WaitCommand((double)delay), chosenAuto);
  }


   



public static void logDriverController() {
  Logger.getInstance().recordOutput("Driver1Controller/leftAxis", controller.getRawAxis(Constants.LEFT_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/RightYAxis", controller.getRawAxis(Constants.RIGHT_Y_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/RightXAxis", controller.getRawAxis(Constants.RIGHT_X_AXIS));
  Logger.getInstance().recordOutput("Driver1Controller/BButton", controller.getRawButtonPressed(2));

}

public static void logOperatorController() {
  Logger.getInstance().recordOutput("Driver2Controller/AButton", controller2.getRawButtonPressed(1));
  Logger.getInstance().recordOutput("Driver2Controller/BButton", controller2.getRawButtonPressed(2));
  Logger.getInstance().recordOutput("Driver2Controller/YButton", controller2.getRawButtonPressed(3));
  Logger.getInstance().recordOutput("Driver2Controller/XButton", controller2.getRawButtonPressed(4));
  Logger.getInstance().recordOutput("Driver2Controller/LeftShoulderButton", controller2.getRawButtonPressed(5));
  Logger.getInstance().recordOutput("Driver2Controller/RightShoulderButton", controller2.getRawButtonPressed(6));
}

public static void logDriverData(){
  logDriverController();
  logOperatorController();

}

}