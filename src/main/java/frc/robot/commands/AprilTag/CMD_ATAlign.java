package frc.robot.commands.AprilTag;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_ATAlign extends CommandBase {
    SUB_AprilTag aprilTag;
    SUB_Drivetrain drive;

    public CMD_ATAlign(SUB_AprilTag at, SUB_Drivetrain dt) {
        aprilTag = at;
        drive = dt;
        addRequirements(at);
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
        
        if (aprilTag.getX() > 0.009) { // turn left
            double turnSpeed = -Math.min(Math.max(aprilTag.getX() * -0.03, -0.5),-0.26);
            drive.setMotorsArcade(0, turnSpeed); // If we are further away, we will turn faster
            SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
            SmartDashboard.putBoolean("aligning", true);
        } else if (aprilTag.getX() < -0.009){ // turn right
            double turnSpeed = -Math.max(Math.min(aprilTag.getX() * -0.03, 0.5),0.26);
            drive.setMotorsArcade(0, turnSpeed); // If we are further away, we will turn faster
            SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
            SmartDashboard.putBoolean("aligning", true);
        }
        else{
            SmartDashboard.putBoolean("aligning", false);
        }
        SmartDashboard.putBoolean("limeAlign", true);
       
    }
       
        
    
    public boolean isFinished(){
        // If we are within 0.0085 degrees of the target, we will stop
        if (Math.abs(aprilTag.getX()) <= 0.05){//  if no target found: limelight.getX() = 0.0
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
