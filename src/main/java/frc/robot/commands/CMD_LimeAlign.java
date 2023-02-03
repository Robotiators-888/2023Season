package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_LimeAlign extends CommandBase {
    SUB_Limelight limelight;
    SUB_Drivetrain drive;

    public CMD_LimeAlign(SUB_Limelight ll, SUB_Drivetrain dt) {
        limelight = ll;
        drive = dt;
        addRequirements(ll);
    } 


    @Override
    public void initialize() {
        // Turns of the breaks to let us start moving
        drive.setBrakeMode(false);
    }
    
    @Override
    public void execute() {    
        // As long as we are more than 0.2 degrees off, we will turns towards target
        if (limelight.getX() > 0.02) {
            drive.setMotorsArcade(0, Math.min(limelight.getX() * -0.025, -0.1)); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.min(limelight.getX() * -0.025, -0.1));
        } else if (limelight.getX() < -0.02){
            drive.setMotorsArcade(0, Math.max(limelight.getX() * 0.025, 0.1)); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + Math.max(limelight.getX() * 0.025, 0.1));
        //  For the second turning
        // As long we are more than 00.2 degrees off, we will turn towards target
        } else if (limelight.getX() < 0.02) {
            drive.setMotorsArcade(0, -0.025); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + -0.025);
        } else if (limelight.getX() > -0.02){
            drive.setMotorsArcade(0, 0.025); // If we are further away, we will turn faster
            System.out.println("X-offset" + limelight.getX());
            System.out.println("SpeedZ " + 0.025);
        }
        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", limelight.getDistance());
        SmartDashboard.putString("Limealign Working", "Yes");
    }
       
        
    
    public boolean isFinished(){
        // If we are within 0.0085 degrees of the target, we will stop
        if (Math.abs(limelight.getX()) <= 0.0085){//  if no target found: limelight.getX() = 0.0
            drive.setBrakeMode(true);
            return true;
        }else{
            return false;
        }
    }
    @Override
    public void end(boolean interrupted) {
        drive.setBrakeMode(true);
    }

}
