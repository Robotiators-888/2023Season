package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ReverseBalance;
import frc.robot.subsystems.*;


public class Autonomous{

    final Field2d field2d = RobotContainer.field2d;
    final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;
    final SUB_AprilTag aprilTag = RobotContainer.apriltag;
    final SUB_Limelight limelight = RobotContainer.limelight;
   // PIDController 


 // ====================================================================
 // Trajectory Config
 // ====================================================================
    TrajectoryConfig configReversed = new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Autonomous.kDriveKinematics)
                 .setReversed(true);

    TrajectoryConfig configForward = new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Autonomous.kDriveKinematics);
 /** 
     * opens and converts path wever file to a trajectory
     * 
     * @param weaverFile sting path to path weaver *.wpilib.json file
     *                   ex.(paths/YourPath.wpilib.json)
     * @return Trajectory onject from path weaver file
     */
    public Trajectory getTrajectory(String weaverFile) {
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(weaverFile);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + weaverFile, ex.getStackTrace());
        }

        return trajectory;
    }

    
    // ====================================================================
    //                          Trajectories
    // ====================================================================
        Trajectory red1_p1 = getTrajectory("paths/output/red1_p1.wpilib.json");
        Trajectory red1_p2 = getTrajectory("paths/output/red1_p2.wpilib.json");
        Trajectory dummyPath = getTrajectory("paths/output/Dummy.wpilib.json");
        Trajectory red3_p3 = getTrajectory("paths/output/Red3_p3.wpilib.json");
        Trajectory red3_p4 = getTrajectory("paths/output/Red3_p4.wpilib.json");
        Trajectory drive_back = getTrajectory("paths/output/DriveBack.wpilib.json");
        Trajectory play1 = getTrajectory("paths/output/play1.wpilib.json");
        Trajectory play1_forwad = getTrajectory("paths/output/play1_forwad.wpilib.json");
        Trajectory balance = getTrajectory("paths/output/Balancing.wpilib.json");

        Trajectory driveToGP_path = getTrajectory("paths/output/DriveToGP.wpilib.json");

        //One Cone Reverse
        Trajectory red1_Backwards = getTrajectory("paths/output/Red1_DriveBack.wpilib.json");
        Trajectory red3_Backwards = getTrajectory("paths/output/Red3_DriveBack.wpilib.json");
        Trajectory blue1_Backwards = getTrajectory("paths/output/Blue1_DriveBack.wpilib.json");
        Trajectory blue3_Backwards = getTrajectory("paths/output/Blue3_DriveBack.wpilib.json");
        

    // ====================================================================
    //                          Auto Sequences
    // ====================================================================

    /**
     * returns ramsete command to drive provided trajectory
     * 
     * @param traj trajectory to follow
     * @return ramsete controller to follow trajectory
     */
    public RamseteCommand getRamsete(Trajectory traj) {
        
        return new RamseteCommand(
                traj, 
                drivetrain::getPose,
                new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.Autonomous.ksVolts, Constants.Autonomous.kvVoltsSecondsPerMeter,
                        Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
                Constants.Autonomous.kDriveKinematics, drivetrain::getWheelSpeeds,
                new PIDController(Constants.Autonomous.kpDriverVelocity, 0, 0),
                new PIDController(Constants.Autonomous.kpDriverVelocity, 0, 0),
                drivetrain::tankDriveVolts, drivetrain);
                

                // Troubleshooting auto :(
                 /*
                return new RamseteCommand(
                    traj, 
                    drivetrain::getPose,
                    new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
                    new SimpleMotorFeedforward(Constants.Autonomous.ksVolts, Constants.Autonomous.kvVoltsSecondsPerMeter,
                            Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
                    Constants.Autonomous.kDriveKinematics, drivetrain::getWheelSpeeds,
                    new PIDController(0, 0, 0),
                    new PIDController(0, 0, 0),
                    drivetrain::tankDriveVolts, drivetrain);
                */
    }
    
    public Command buildScoringSequence(){
     return new SequentialCommandGroup(
        new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)),
         new SequentialCommandGroup(
          new WaitCommand(2.5), 
          new InstantCommand(() -> gripper.openConeGripper(), gripper))),
          new SequentialCommandGroup(
            new WaitCommand(1), 
            new InstantCommand(()->gripper.closeConeGripper())), 
            new SequentialCommandGroup(
                new WaitCommand(0.5), 
                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower))));
    }

    public Command buildPickUpSequence(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(()->tower.setTargetPosition(Constants.Arm.kIntakePosition, tower)),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new InstantCommand(()->gripper.openConeGripper())
                )
            ),
            new WaitCommand(1.5),
            new InstantCommand(()->gripper.closeConeGripper()),
            new WaitCommand(0.5),
            new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower))
        );
    }

    public Command buildAutoBalanceSequence(){
        return new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(0.75, 0);}, drivetrain).withTimeout(1.5),
            new ReverseBalance(drivetrain)
        );
    }

    public Command buildReverseAutoBalanceSequence(){
        return new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.75, 0);}, drivetrain).withTimeout(1.5),
            new ReverseBalance(drivetrain)
        );
    }
    // Command autoBalanceSequence = new SequentialCommandGroup(
    //     new RunCommand(()->drivetrain.setMotorsTank(0.65, 0.65), drivetrain)
    //     .until(()->(drivetrain.getPitch() <= -9 && drivetrain.getPitch() > ) )
        
    // );

    
    Command turn180Degree() {
        
        return new RunCommand(()->drivetrain.turn180Degree(), drivetrain)
        .until(()->(drivetrain.getAngle() < -175 || drivetrain.getAngle() > 175))
        .withTimeout(2).andThen(()->SmartDashboard.putBoolean("Is turning", false));
    }

    public Command buildAprilTagPlacementSequence(){
        return new SequentialCommandGroup(
          new RunCommand(() -> {aprilTag.switchapipeline(1);}, aprilTag),
          new RunCommand(() -> {aprilTag.aprilAlign();}, aprilTag).until(() -> (aprilTag.getX() <= 0.05)),
          new RunCommand(() -> {aprilTag.aprilDrive();}, aprilTag).until(() -> (aprilTag.getDistance() <= 12)),
          new RunCommand(() -> {aprilTag.aprilAlign();}, aprilTag).until(() -> (aprilTag.getX() <= 0.05)),
          new ParallelCommandGroup(
            new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)),
             new SequentialCommandGroup(
              new WaitCommand(2.5), 
              new InstantCommand(() -> gripper.openConeGripper(), gripper))),
              new SequentialCommandGroup(
                new WaitCommand(1), 
                new InstantCommand(()->gripper.closeConeGripper())), 
                new SequentialCommandGroup(
                    new WaitCommand(0.5), 
                    new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower))));
      }

      public Command buildLimelightPlacementSequence(){
        return new SequentialCommandGroup(
          new RunCommand(() -> {limelight.switchapipeline(1);}, limelight),
          new RunCommand(() -> {limelight.setLed(3);}, limelight),
          new WaitCommand(1),
          new RunCommand(() -> {limelight.limelightAlign();}, limelight).until(() -> (limelight.getX() <= 0.05)),
          new RunCommand(() -> {limelight.limelightDrive();}, limelight).until(() -> (limelight.getDistance() <= 24.5)),
          new RunCommand(() -> {limelight.limelightAlign();}, limelight).until(() -> (limelight.getX() <= 0.05)),
          new RunCommand(() -> {limelight.setLed(1);}, limelight),
          new ParallelCommandGroup(
            new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)),
             new SequentialCommandGroup(
              new WaitCommand(2.5), 
              new InstantCommand(() -> gripper.openConeGripper(), gripper))),
              new SequentialCommandGroup(
                new WaitCommand(1), 
                new InstantCommand(()->gripper.closeConeGripper())), 
                new SequentialCommandGroup(
                    new WaitCommand(0.5), 
                    new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower))));
      }



    // ==================================================================== 
    //                          Auto Routines
    // ====================================================================
    

    Command red1_Score1(){
        field2d.getObject("trajectory").setTrajectory(red1_p1);   
        return new SequentialCommandGroup(
           new InstantCommand(()->drivetrain.setPosition(red1_p1.getInitialPose())),
            buildScoringSequence(),
            getRamsete(red1_p1),
            buildPickUpSequence(),
            new InstantCommand(()->field2d.getObject("trajectory").setTrajectory(red1_p2)),
            getRamsete(red1_p2),
            buildScoringSequence()
            );
    } 

    Command driveBack(){
        field2d.getObject("trajectory").setTrajectory(drive_back);   
        return new SequentialCommandGroup(
            new InstantCommand(()->drivetrain.setPosition(drive_back.getInitialPose())),
            getRamsete(drive_back));
    }

    Command scoreDriveBack(){
        field2d.getObject("trajectory").setTrajectory(dummyPath);   
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(dummyPath.getInitialPose())),
            getRamsete(dummyPath));
    }
    
    Command forwardsScoreThenAutoBalance(){
        drivetrain.resetAngle();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new WaitCommand(1),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(0.2),
            turn180Degree(),
            buildAutoBalanceSequence()
        );
    }
    Command red3_Mid_2GP(){
       // field2d.getObject("trajectory").setTrajectory(red3_p3);   
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(red3_p3.getInitialPose())),
            getRamsete(red3_p3)
        );
    }

    Command backwardsScoreThenAutoBalance(){
        drivetrain.leftPrimary.setIdleMode(IdleMode.kBrake);
        drivetrain.leftSecondary.setIdleMode(IdleMode.kBrake);
        drivetrain.rightPrimary.setIdleMode(IdleMode.kBrake);
        drivetrain.rightSecondary.setIdleMode(IdleMode.kBrake);
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)),
         new SequentialCommandGroup(
          new WaitCommand(2.5), 
          new InstantCommand(() -> gripper.openConeGripper(), gripper))),
          new SequentialCommandGroup(
            new WaitCommand(1), 
            new InstantCommand(()->gripper.closeConeGripper())),
        new SequentialCommandGroup(
            //new RunCommand(()->drivetrain.setMotorsTank(0.4, 0.4))),
           new InstantCommand(()->drivetrain.setPosition(balance.getInitialPose()))),
           //buildAutoBalanceSequence()
           buildReverseAutoBalanceSequence()
        );
        
    }

    Command Red1_Cone_DB(){
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(red1_Backwards.getInitialPose())),
            getRamsete(red1_Backwards)
        );
    }

    Command Blue1_Cone_DB(){
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(blue1_Backwards.getInitialPose())),
            getRamsete(blue1_Backwards)
        );
    }

    Command Blue3_Cone_DB(){
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(blue3_Backwards.getInitialPose())),
            getRamsete(blue3_Backwards)
        );
    }
    Command Red3_Cone_DB(){
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(red3_Backwards.getInitialPose())),
            getRamsete(red3_Backwards)
        );
    }

    Command Cone_AutoBalance(){
        return new SequentialCommandGroup(
            new InstantCommand(()->SmartDashboard.putBoolean("Is turning", false)),

            new InstantCommand(()->drivetrain.zeroHeading()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            turn180Degree(),
            buildAutoBalanceSequence() //This is the improved balance conditional
        );
    }

    Command DriveToGamePiece(){
        return new SequentialCommandGroup(
            new InstantCommand(()->drivetrain.setPosition(driveToGP_path.getInitialPose())),
            getRamsete(driveToGP_path)
        );
    }




}
