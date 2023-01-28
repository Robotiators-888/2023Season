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
    //  in inches
    

    @Override
    public void initialize() {
        drive.setBrakeMode(false);
    }
    
    @Override
    public void execute() {
        double distance = limelight.getDistance();    
        if (limelight.getX() > 0.2) {
            drive.setMotorsArcade(0, Math.min(limelight.getX() * -0.02, -0.3));
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.min(limelight.getX() * -0.02, -0.3));
        } else if (limelight.getX() < -0.2){
            drive.setMotorsArcade(0, Math.max(limelight.getX() * 0.02, 0.3));
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.max(limelight.getX() * 0.02, 0.3));
        }
        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putString("Limealign Working", "Yes");
    }
       
        
    
    public boolean isFinished(){
        if (Math.abs(limelight.getX()) < 0.25){//  if no target found: limelight.getX() = 0.0
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
