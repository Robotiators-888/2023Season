package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.*;


public class Autonomous{

    final Field2d field2d = RobotContainer.field2d;
    final SUB_Gripper gripper = RobotContainer.gripper;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;

 // ====================================================================
 // Trajectories
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

    /**
     * returns ramsete command to drive provided trajectory
     * 
     * @param traj trajectory to follow
     * @return ramsete controller to follow trajectory
     */
    public RamseteCommand getRamset(Trajectory traj) {
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

    Trajectory red1_p1 = getTrajectory("Pathweaver/output/red1_p1.wpilib.json");
    Trajectory red1_p2 = getTrajectory("Pathweaver/output/red1_p2.wpilib.json");





}