package frc.robot.commands.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_LLAlign extends CommandBase {
    SUB_Limelight limelight;
    SUB_Drivetrain drive;

    public CMD_LLAlign(SUB_Limelight ll, SUB_Drivetrain dt) {
        limelight = ll;
        drive = dt;
        addRequirements(ll);
        addRequirements(dt);
    } 


    @Override
    public void initialize() {
        // Turns of the breaks to let us start moving
        drive.setBrakeMode(false);
        SmartDashboard.putNumber("Limelight turnspeed: ", 0);
    }   
    
    @Override
    public void execute() {    
        // As long as we are more than 0.009 degrees off, we will turns towards target
        
        if (limelight.getX() > 0.009) { // turn left
            double turnSpeed = -Math.min(Math.max(limelight.getX() * -0.03, -0.5),-0.275);
            drive.driveArcadeSquared(0, turnSpeed); // If we are further away, we will turn faster
            SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
            SmartDashboard.putBoolean("aligning", true);
        } else if (limelight.getX() < -0.009){ // turn right
            double turnSpeed = -Math.max(Math.min(limelight.getX() * -0.03, 0.5),0.275);
            drive.driveArcadeSquared(0, turnSpeed); // If we are further away, we will turn faster
            SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
            SmartDashboard.putBoolean("aligning", true);
        }
        else{
            SmartDashboard.putBoolean("aligning", false);
        }
        SmartDashboard.putBoolean("limeAlign", true);
       
    }
       
        
    
    public boolean isFinished(){
        // If we are within 0.05 degrees of the target, we will stop
        if (Math.abs(limelight.getX()) <= 0.05){//  if no target found: limelight.getX() = 0.0
            return true;
        }else{
            return false;
        }
    }
    @Override
    public void end(boolean interrupted) {
        drive.setBrakeMode(true);
        SmartDashboard.putNumber("Limelight turnspeed: ", 0);
    }

}
