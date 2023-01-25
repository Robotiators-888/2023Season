package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeAlign extends CommandBase {
    Limelight limelight;
    Drivetrain drive;

    public LimeAlign(Limelight ll, Drivetrain dt) {
        limelight = ll;
        drive = dt;
    } 
    //  in inches
    

    @Override
    public void initialize() {
        limelight.setLed(3);
    }
    
    @Override
    public void execute() {
        double distance = limelight.getDistance();
        if (limelight.getTv()) {
            if (distance > 25) {
             
                drive.setMotorsArcade(0, .0);
            }
        }

        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", distance);

    }
    @Override
    public void end(boolean interrupted) {
        limelight.setLed(0);
    }

}
