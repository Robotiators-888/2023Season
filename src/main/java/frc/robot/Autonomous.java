package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

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
import frc.libs.EventMap;
import frc.robot.commands.ReverseBalance;
import frc.robot.commands.UpAndOverBalance;
import frc.robot.subsystems.*;


public class Autonomous{

    final Field2d field2d = RobotContainer.field2d;
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
        Trajectory RED_driveToGP_path = getTrajectory("paths/output/RED_DriveToGP.wpilib.json");
        Trajectory RED_forward_GP_path = getTrajectory("paths/output/RED_Forward_GP.wpilib.json");
        Trajectory RED_CABLE_TO_GP_path = getTrajectory("paths/output/RED_CABLE_TO_GP.wpilib.json");
        Trajectory RED_CABLE_PICKUP_path = getTrajectory("paths/output/RED_CABLE_PICKUP.wpilib.json");

        Trajectory BLUE_driveToGP_path = getTrajectory("paths/output/BLUE_DriveToGP.wpilib.json");
        Trajectory BLUE_forward_GP_path = getTrajectory("paths/output/BLUE_Forward_GP.wpilib.json");
        Trajectory BLUE_CABLE_TO_GP_path = getTrajectory("paths/output/BLUE_CABLE_TO_GP.wpilib.json");
        Trajectory BLUE_CABLE_PICKUP_path = getTrajectory("paths/output/BLUE_CABLE_PICKUP.wpilib.json");

        Trajectory RED_ChargeGP_path = getTrajectory("paths/output/RED_ChargeGP.wpilib.json");

        Trajectory red1_Backwards = getTrajectory("paths/output/Red1_DriveBack.wpilib.json");
        Trajectory red3_Backwards = getTrajectory("paths/output/Red3_DriveBack.wpilib.json");
        Trajectory blue1_Backwards = getTrajectory("paths/output/Blue1_DriveBack.wpilib.json");
        Trajectory blue3_Backwards = getTrajectory("paths/output/Blue3_DriveBack.wpilib.json");

        PathPlannerTrajectory dummyPath = PathPlannerBase.getTrajectory("DummyPath", true);
        PathPlannerTrajectory dummyStop = PathPlannerBase.getTrajectory("DummyStop", true);
        

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
            .until(()->(drivetrain.getHeading() < -0.01 && drivetrain.getHeading() > 0.01))
            .withTimeout(1.25);
    }

    

      



    // ==================================================================== 
    //                          Auto Routines
    // ====================================================================
    

    public Command placeOneCone(){
        stateManager.setCone();
        return buildScoringSequence();
    }

    public Command placeOneCube(){
        stateManager.setCube();
        return buildScoringSequence();
    }

    public Command dummyCommand(){
        stateManager.setCube();
        return new SequentialCommandGroup(
            buildScoringSequence(),
            //PathPlannerBase.getRamsete(dummyPath, true)
            PathPlannerBase.generateAuto(new HashMap<String, Command>(), dummyPath)
        );
    }

    public Command Red1_Cone_DB(){
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
            new InstantCommand(()->stateManager.setCube()),
            buildScoringSequence(),
            new RunCommand(()->{drivetrain.setMotorsArcade(-0.3, 0);}, drivetrain).withTimeout(.5),
            new InstantCommand(()->drivetrain.zeroHeading()),
            new RunCommand(()-> drivetrain.turnToTheta(180), drivetrain)
            .until(()->(drivetrain.getHeading() < -173 || drivetrain.getHeading() > 173))
            .withTimeout(2.5),
            new RunCommand(()->{drivetrain.setMotorsArcade(0.75, 0);}, drivetrain).until(()->drivetrain.getPitch() > 10),
            new RunCommand(()->drivetrain.setMotorsArcade(0.5, 0), drivetrain).until(()->drivetrain.getPitch() < -7.5),
            new RunCommand(()->{drivetrain.setMotorsArcade(0.5, 0);}, drivetrain).withTimeout(1.15),
            turnToZero(),
            new SequentialCommandGroup(
            new RunCommand(()->{drivetrain.setMotorsArcade(0.75, 0);}, drivetrain).withTimeout(1.35),
            new UpAndOverBalance(drivetrain)
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
            new RunCommand(()->{drivetrain.setMotorsArcade(0.6, 0);}, drivetrain).until(()->drivetrain.getPitch() > 10),
            new RunCommand(()->drivetrain.setMotorsArcade(0.45, 0), drivetrain).until(()->drivetrain.getPitch() < -7.5),
            new RunCommand(()->{drivetrain.setMotorsArcade(0.5, 0);}, drivetrain).withTimeout(1.15),
            new SequentialCommandGroup(
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kGroundPosition(), tower)),
                new InstantCommand(()->stateManager.intakeRoller()),
                new WaitCommand(.5)),
            new InstantCommand(()->drivetrain.setPosition(RED_ChargeGP_path.getInitialPose())),
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
                turnToZero()
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
            RED_DriveToGamePiece()
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
                new InstantCommand(() -> tower.setTargetPosition(stateManager.kHomePosition(), tower))),
                //new InstantCommand(()->tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
                new InstantCommand(()->stateManager.stopRoller())
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
                turnToZero()
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
            BLUE_DriveToGamePiece()
        );
    }


}
