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
import frc.robot.commands.ForwardBalance;
import frc.robot.commands.ReverseBalance;
import frc.robot.subsystems.*;


public class Autonomous{

    final Field2d field2d = RobotContainer.field2d;
    final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;
    edu.wpi.first.wpilibj.Timer balanceTime = new edu.wpi.first.wpilibj.Timer();

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
        Trajectory red1_p1 = getTrajectory("paths/output/Red1_p1.wpilib.json");
        Trajectory red1_p2 = getTrajectory("paths/output/Red1_p2.wpilib.json");
        Trajectory dummyPath = getTrajectory("paths/output/Dummy.wpilib.json");
        Trajectory balance = getTrajectory("paths/output/Balancing.wpilib.json");

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

    //Drives forward onto charge station then auto balances
    public Command buildForwardAutoBalanceSequence(){
        return new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(0.75, 0);}, drivetrain).withTimeout(1.5),
            new ForwardBalance(drivetrain)
        );
    }

    
    // public Command buildReverseAutoBalanceSequence(){
    //     return new SequentialCommandGroup(
    //         new RunCommand(()->{drivetrain.setMotorsArcade(-0.75, 0);}, drivetrain).withTimeout(1.5),
    //         new ReverseBalance(drivetrain)
    //     );
    // }
    
    // turns a little less than 180 degrees for AutoBalance to get ont charge station at slight angle
    Command buildTurnTo180DegreeSequence() {
        return new RunCommand(()->drivetrain.turnTo180Degree(), drivetrain)
        .until(()->(drivetrain.getAngle() < -175 || drivetrain.getAngle() > 175))
        .withTimeout(2).andThen(()->SmartDashboard.putBoolean("Is turning", false));
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
            new InstantCommand(()->field2d.getObject("trajectory").setTrajectory(red1_p2)            ),
            getRamsete(red1_p2),
            buildScoringSequence()
            );
    } 

    Command driveBack(){
        field2d.getObject("trajectory").setTrajectory(dummyPath);   
        return new SequentialCommandGroup(
            new InstantCommand(()->drivetrain.setPosition(dummyPath.getInitialPose())),
            getRamsete(dummyPath));
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

    Command Cone_Turn180_AutoBalance(){
        return new SequentialCommandGroup(
            new InstantCommand(()->SmartDashboard.putBoolean("Is turning", false)),
            new InstantCommand(()->drivetrain.zeroHeading()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            new WaitCommand(0.5), 
            buildTurnTo180DegreeSequence(),
            buildForwardAutoBalanceSequence() //This is the improved balance conditional
        );
    }

    Command DriveForward_AutoBalance(){
        drivetrain.resetAngle();
        return new SequentialCommandGroup(
            buildTurnTo180DegreeSequence(),
            buildForwardAutoBalanceSequence()
        );
    }

    // Command backwardsScoreThenAutoBalance(){
    //     drivetrain.leftPrimary.setIdleMode(IdleMode.kBrake);
    //     drivetrain.leftSecondary.setIdleMode(IdleMode.kBrake);
    //     drivetrain.rightPrimary.setIdleMode(IdleMode.kBrake);
    //     drivetrain.rightSecondary.setIdleMode(IdleMode.kBrake);
    //     return new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //     new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringPosition, tower)),
    //      new SequentialCommandGroup(
    //       new WaitCommand(2.5), 
    //       new InstantCommand(() -> gripper.openConeGripper(), gripper))),
    //       new SequentialCommandGroup(
    //         new WaitCommand(1), 
    //         new InstantCommand(()->gripper.closeConeGripper())),
    //     new SequentialCommandGroup(
    //         //new RunCommand(()->drivetrain.setMotorsTank(0.4, 0.4))),
    //        new InstantCommand(()->drivetrain.setPosition(balance.getInitialPose()))),
    //        //buildAutoBalanceSequence()
    //        buildReverseAutoBalanceSequence()
    //     );
        
    // }

    // Driveforward AutoBalance
    // Cone Turn180 AutoBalance
    // DriveBackwards AutoBalance Backwards




}
