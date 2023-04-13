package frc.robot.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUB_Drivetrain;



public class PathPlannerBase {
    final static SUB_Drivetrain drivetrain = RobotContainer.drivetrain;

    final static PathConstraints constraints = new PathConstraints(8, 5);

    //  |------------------------------------------------------------------|
    //  |                          Trajectories                            |
    //  |__________________________________________________________________|

    PathPlannerTrajectory ds3_driveback = getTrajectory("DS3_DriveBack", true); 
    PathPlannerTrajectory dummyPath = getTrajectory("DummyPath", true);


    public static PathPlannerTrajectory getTrajectory(String plannerFile, boolean reversed) {
        PathPlannerTrajectory trajectoryPath;
            trajectoryPath = PathPlanner.loadPath(plannerFile, constraints, reversed); //Filesystem.getDeployDirectory().toPath().resolve(plannerFile);
        

        return trajectoryPath;
    }
    
    public static PPRamseteCommand getRamsete(PathPlannerTrajectory traj, boolean resetOdometry) {
        
        if (resetOdometry) {
            drivetrain.resetOdometry(traj.getInitialHolonomicPose());  
        }

        return new PPRamseteCommand(
            traj, 
                drivetrain::getPose,
                new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.Autonomous.ksVolts, Constants.Autonomous.kvVoltsSecondsPerMeter,
                        Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
                Constants.Autonomous.kDriveKinematics, drivetrain::getWheelSpeeds,
                new PIDController(Constants.Autonomous.kpDriverVelocity, 0, 0),
                new PIDController(Constants.Autonomous.kpDriverVelocity, 0, 0),
                drivetrain::tankDriveVolts, 
                true, 
                drivetrain);
    }

    

}
