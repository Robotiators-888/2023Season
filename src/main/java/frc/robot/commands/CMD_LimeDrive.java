package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_LimeDrive extends CommandBase {
    SUB_Limelight limelight;
    SUB_Drivetrain drive;

    public CMD_LimeDrive(SUB_Limelight ll, SUB_Drivetrain dt) {
        limelight = ll;
        drive = dt;
        addRequirements(ll);
    } 
    //  in inches
    

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // We are using the Y value, because the Y value will change as you get closer or further
        // We got a pretty accurate value for what the y value should be in order for us to get as close as we can
        double distance = limelight.getY();
        // We check if we can even find one to begin with, or it will be impossible to turn to something tha tdoesn't exist
        if(limelight.getTv()){
            // We found that -5.1 is the correct number for Y, so as long as we don't reach it, we will drive forward
            if (distance >= -5.1) {
                drive.setMotorsArcade(Math.min(1-Math.abs(limelight.getY() * 0.05), 0.2), 0.0); // If we are further away, we drive faster 
                System.out.println(distance);
            }
            // -6.02 is the correct value for Y, so as long as we don't reach it, we will drive forward
            if (distance >= -6.02) {
                drive.setMotorsArcade( 0.075, 0.0); // If we are further away, we drive faster 
                System.out.println(distance);
            }
        }
        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", distance);
    }
    public boolean isFinished(){
        // If we reach the place, then we stop
        if(limelight.getY() <= -5.1 || limelight.getY() <= -6.02){ // First part of the statement is for the first run and the second part is for the second run
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