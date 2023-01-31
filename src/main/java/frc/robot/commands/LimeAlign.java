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
        addRequirements(ll, dt);
    } 


    @Override
    public void initialize() {
        // Turns of the breaks to let us start moving
        drive.setBrakeMode(false);
    }
    
    @Override
    public void execute() {    
        // As long as we are more than 0.2 degrees off, we will turns towards it
        if (limelight.getX() > 0.2) {
            drive.setMotorsArcade(0, Math.min(limelight.getX() * -0.02, -0.3)); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.min(limelight.getX() * -0.02, -0.3));
        } else if (limelight.getX() < -0.2){
            drive.setMotorsArcade(0, Math.max(limelight.getX() * 0.02, 0.3)); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.max(limelight.getX() * 0.02, 0.3));
        }
        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", limelight.getDistance());
        SmartDashboard.putString("Limealign Working", "Yes");
    }
       
        
    
    public boolean isFinished(){
        // If we are within 0.2 degrees of the target, we will stop
        if (Math.abs(limelight.getX()) < 0.2){//  if no target found: limelight.getX() = 0.0
            System.out.println("limealign finished: "+limelight.getX());
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        drive.setBrakeMode(true);
    }

}
