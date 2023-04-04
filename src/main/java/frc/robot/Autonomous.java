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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ReverseBalance;
import frc.robot.subsystems.*;


public class Autonomous{

    final Field2d field2d = RobotContainer.field2d;
    //final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;
    final SUB_Roller roller = RobotContainer.roller;
    final StateManager stateManager = RobotContainer.stateManager;

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

        Trajectory RED_driveToGP_path = getTrajectory("paths/output/RED_DriveToGP.wpilib.json");
        Trajectory RED_forward_GP_path = getTrajectory("paths/output/RED_Forward_GP.wpilib.json");
        Trajectory RED_CABLE_TO_GP_path = getTrajectory("paths/output/RED_CABLE_TO_GP.wpilib.json");
        Trajectory RED_CABLE_PICKUP_path = getTrajectory("paths/output/RED_CABLE_PICKUP.wpilib.json");

        Trajectory BLUE_driveToGP_path = getTrajectory("paths/output/BLUE_DriveToGP.wpilib.json");
        Trajectory BLUE_forward_GP_path = getTrajectory("paths/output/BLUE_Forward_GP.wpilib.json");
        Trajectory BLUE_CABLE_TO_GP_path = getTrajectory("paths/output/BLUE_CABLE_TO_GP.wpilib.json");
        Trajectory BLUE_CABLE_PICKUP_path = getTrajectory("paths/output/BLUE_CABLE_PICKUP.wpilib.json");

        Trajectory RED_ChargeGP_path = getTrajectory("paths/output/RED_ChargeGP.wpilib.json");

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




    // ====================================================================
    //                          Auto Sequences
    // ====================================================================

    // public Command buildScoringSequence(){
    //     return new SequentialCommandGroup(
    //        new ParallelCommandGroup(
    //        new InstantCommand(() -> tower.setTargetPosition(Constants.Arm.kScoringConePosition, tower)),
    //         new SequentialCommandGroup(
    //          new WaitCommand(2.5), 
    //          new InstantCommand(() -> gripper.openGripper(), gripper))),
    //          new SequentialCommandGroup(
    //            new WaitCommand(1), 
    //            new InstantCommand(()->gripper.closeGripper())), 
    //            new SequentialCommandGroup(
    //                new WaitCommand(0.5), 
    //                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower))));
    //    }

    public Command buildScoringSequence(){
        return new SequentialCommandGroup(
                    new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                    new WaitCommand(2),
                    new InstantCommand(()-> stateManager.outtakeRoller()),
                new SequentialCommandGroup(
                   new WaitCommand(1),
                   new InstantCommand(()->stateManager.stopRoller()),
                    new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)))
                    );

        
    }

    

    public Command buildAutoBalanceSequence(){
        return new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(0.7, 0);}, drivetrain).withTimeout(1.5),
            new ReverseBalance(drivetrain)
        );
    }

    public Command buildReverseAutoBalanceSequence(){
        return new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.75, 0);}, drivetrain).withTimeout(1.5),
            new ReverseBalance(drivetrain)
        );
    }

    
    Command turn180Degree() {
        
        // return new RunCommand(()->drivetrain.turn180Degree(), drivetrain)
        // .until(()->(drivetrain.getHeading() < -180 || drivetrain.getHeading() > 180))
        // .withTimeout(2.5).andThen(()->SmartDashboard.putBoolean("Is turning", false));

        return new RunCommand(()-> drivetrain.turnToTheta(180), drivetrain)
            .until(()->(drivetrain.getHeading() < -175 || drivetrain.getHeading() > 175))
            .withTimeout(2.5);
    }

    Command turnToZero(){
        return new RunCommand(()-> drivetrain.turnToTheta(0), drivetrain)
            .until(()->(drivetrain.getHeading() < -0.0 && drivetrain.getHeading() > 0))
            .withTimeout(2.5);
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
            //buildPickUpSequence(),
            new InstantCommand(()->field2d.getObject("trajectory").setTrajectory(red1_p2)            ),
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
        new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
         new SequentialCommandGroup(
          new WaitCommand(2.5), 
          new SequentialCommandGroup(
            new WaitCommand(1) 
            //new InstantCommand(()->gripper.closeGripper()
            )),
        new SequentialCommandGroup(
            //new RunCommand(()->drivetrain.setMotorsTank(0.4, 0.4))),
           new InstantCommand(()->drivetrain.setPosition(balance.getInitialPose()))),
           //buildAutoBalanceSequence()
           buildReverseAutoBalanceSequence()
        ));
        
    }

    Command Red1_Cone_DB(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(red1_Backwards.getInitialPose())),
            getRamsete(red1_Backwards)
        );
    }

    Command Blue1_Cone_DB(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(blue1_Backwards.getInitialPose())),
            getRamsete(blue1_Backwards)
        );
    }

    Command Blue3_Cone_DB(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(blue3_Backwards.getInitialPose())),
            getRamsete(blue3_Backwards)
        );
    }
    Command Red3_Cone_DB(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            new InstantCommand(()->drivetrain.setPosition(red3_Backwards.getInitialPose())),
            getRamsete(red3_Backwards)
        );
    }

    Command Cube_AutoBalance(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            new InstantCommand(()->SmartDashboard.putBoolean("Is turning", false)),

            new InstantCommand(()->drivetrain.zeroHeading()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            new WaitCommand(.5),
            turn180Degree(),
            buildAutoBalanceSequence() //This is the improved balance conditional
        );
    }
    
    Command UpAndOver(){
        drivetrain.zeroHeading();
        return new SequentialCommandGroup(
        //     new InstantCommand(()->stateManager.setCube()),
        //     buildScoringSequence(),
        //     new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
        //     new InstantCommand(()->drivetrain.zeroHeading()),
        //     turn180Degree(),
        //     new RunCommand(()->{drivetrain.setMotorsArcade(0.6, 0);}, drivetrain).until(()->drivetrain.getAngle() > 10),//.withTimeout(1.5),
        //     new RunCommand(()->drivetrain.setMotorsArcade(0.5, 0), drivetrain).withTimeout(3),
        //     turnToZero(),
        //     new SequentialCommandGroup(
        //     new RunCommand(()->{drivetrain.setMotorsArcade(0.4, 0);}, drivetrain).withTimeout(.75),
        //     new ReverseBalance(drivetrain)
        // )
            new InstantCommand(()->stateManager.setCube()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            new InstantCommand(()->drivetrain.zeroHeading()),
            turn180Degree(),
            new RunCommand(()->{drivetrain.setMotorsArcade(0.6, 0);}, drivetrain).until(()->drivetrain.getAngle() > 10),
            new RunCommand(()->drivetrain.setMotorsArcade(0.5, 0), drivetrain).until(()->drivetrain.getAngle() > -10),
            turnToZero(),
            new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(0.4, 0);}, drivetrain).withTimeout(.75),
             new ReverseBalance(drivetrain)
            )
        );
    }

    Command TwoGPUpAndOver(){
        drivetrain.zeroHeading();

        return new SequentialCommandGroup(
            new InstantCommand(()->stateManager.setCube()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            new InstantCommand(()->drivetrain.zeroHeading()),
            turn180Degree(),
            new RunCommand(()->{drivetrain.setMotorsArcade(0.6, 0);}, drivetrain).until(()->drivetrain.getAngle() > 10),
            new RunCommand(()->drivetrain.setMotorsArcade(0.5, 0), drivetrain).until(()->drivetrain.getAngle() > -10),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(.5)),
            getRamsete(RED_ChargeGP_path),
            new SequentialCommandGroup(
                new InstantCommand(()->stateManager.stopRoller()),
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
            turnToZero(),
            buildAutoBalanceSequence()

        );
    }

    Command RED_DriveToGamePiece(){
        stateManager.setCube();
        drivetrain.zeroHeading();
        return new SequentialCommandGroup(
            new InstantCommand(()->stateManager.setCube()),
            new InstantCommand(()->drivetrain.zeroHeading()),
            new SequentialCommandGroup(
                    new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                    new WaitCommand(1.75),
                    new InstantCommand(()-> stateManager.outtakeRoller()),
                new SequentialCommandGroup(
                   new WaitCommand(0.15),
                   new InstantCommand(()->stateManager.stopRoller()))),
                   new InstantCommand(()->drivetrain.setPosition(RED_driveToGP_path.getInitialPose())),
            new ParallelCommandGroup(
                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)),
                getRamsete(RED_driveToGP_path)),
            turn180Degree(),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(.5)),
            getRamsete(RED_forward_GP_path),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                //new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
                new InstantCommand(()->tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                new InstantCommand(()->stateManager.stopRoller()))
        );
    }

    Command RED_DriveToGamePieceCable(){
        stateManager.setCube();
        drivetrain.zeroHeading();
        return new SequentialCommandGroup(
            new InstantCommand(()->stateManager.setCube()),
            new InstantCommand(()->drivetrain.zeroHeading()),
            new SequentialCommandGroup(
                    new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                    new WaitCommand(1.75),
                    new InstantCommand(()-> stateManager.outtakeRoller()),
                new SequentialCommandGroup(
                   new WaitCommand(0.15),
                   new InstantCommand(()->stateManager.stopRoller()))),
                   new InstantCommand(()->drivetrain.setPosition(RED_CABLE_TO_GP_path.getInitialPose())),
            new ParallelCommandGroup(
                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)),
                getRamsete(RED_CABLE_TO_GP_path)),
            turn180Degree(),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(1)),
            getRamsete(RED_CABLE_PICKUP_path),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                //new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
                new InstantCommand(()->tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                new InstantCommand(()->stateManager.stopRoller()))
        );
    }

    Command REDTwoCubeCable(){
        return new SequentialCommandGroup(
                RED_DriveToGamePiece(),
                turnToZero(),
                new RunCommand(()->drivetrain.setMotorsArcade(0.855, 0), drivetrain).until(()->Timer.getMatchTime() < .15),
                new InstantCommand(()-> stateManager.outtakeRoller())
        );
    }

    Command REDTwoPieceSPIT(){
        return new SequentialCommandGroup(
            RED_DriveToGamePiece(),
            turnToZero(),
            new RunCommand(()->drivetrain.setMotorsArcade(0.855, 0), drivetrain).until(()->Timer.getMatchTime() < .15),
            new InstantCommand(()-> stateManager.outtakeRoller())
        );
    }

    Command REDTwoPieceHOLD(){
        return new SequentialCommandGroup(
            RED_DriveToGamePiece(),
            turnToZero(),
            new RunCommand(()->drivetrain.setMotorsArcade(0.65, 0), drivetrain).until(()->Timer.getMatchTime() < .15)
        );
    }

    Command BLUE_DriveToGamePiece(){
        stateManager.setCube();
        drivetrain.zeroHeading();
        return new SequentialCommandGroup(
            new InstantCommand(()->stateManager.setCube()),
            new InstantCommand(()->drivetrain.zeroHeading()),
            new SequentialCommandGroup(
                    new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                    new WaitCommand(1.75),
                    new InstantCommand(()-> stateManager.outtakeRoller()),
                new SequentialCommandGroup(
                   new WaitCommand(0.15),
                   new InstantCommand(()->stateManager.stopRoller()))),
                   new InstantCommand(()->drivetrain.setPosition(BLUE_driveToGP_path.getInitialPose())),
            new ParallelCommandGroup(
                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)),
                getRamsete(BLUE_driveToGP_path)),
            turn180Degree(),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(.5)),
            getRamsete(BLUE_forward_GP_path),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                //new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
                new InstantCommand(()->tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                new InstantCommand(()->stateManager.stopRoller()))
        );
    }

    Command BLUE_DriveToGamePieceCable(){
        stateManager.setCube();
        drivetrain.zeroHeading();
        return new SequentialCommandGroup(
            new InstantCommand(()->stateManager.setCube()),
            new InstantCommand(()->drivetrain.zeroHeading()),
            new SequentialCommandGroup(
                    new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                    new WaitCommand(1.75),
                    new InstantCommand(()-> stateManager.outtakeRoller()),
                new SequentialCommandGroup(
                   new WaitCommand(0.15),
                   new InstantCommand(()->stateManager.stopRoller()))),
                   new InstantCommand(()->drivetrain.setPosition(BLUE_CABLE_TO_GP_path.getInitialPose())),
            new ParallelCommandGroup(
                new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)),
                getRamsete(BLUE_CABLE_TO_GP_path)),
            turn180Degree(),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(1)),
            getRamsete(BLUE_CABLE_PICKUP_path),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                //new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
                new InstantCommand(()->tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                new InstantCommand(()->stateManager.stopRoller()))
        );
    }

    Command BLUETwoCubeCable(){
        return new SequentialCommandGroup(
                BLUE_DriveToGamePiece(),
                turnToZero(),
                new RunCommand(()->drivetrain.setMotorsArcade(0.86, 0), drivetrain).until(()->Timer.getMatchTime() < .15),
                new InstantCommand(()-> stateManager.outtakeRoller())
        );
    }

    Command BLUETwoPieceSPIT(){
        return new SequentialCommandGroup(
            BLUE_DriveToGamePiece(),
            turnToZero(),
            new RunCommand(()->drivetrain.setMotorsArcade(0.845, 0), drivetrain).until(()->Timer.getMatchTime() < .15),
            new InstantCommand(()-> stateManager.outtakeRoller())
        );
    }

    Command BLUETwoPieceHOLD(){
        return new SequentialCommandGroup(
            BLUE_DriveToGamePiece(),
            turnToZero(),
            new RunCommand(()->drivetrain.setMotorsArcade(0.65, 0), drivetrain).until(()->Timer.getMatchTime() < .15)
        );
    }


}
