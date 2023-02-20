package frc.robot.commands.AprilTag;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_ATDrive extends CommandBase {
    SUB_AprilTag aprilTag;
    SUB_Drivetrain drive;

    public CMD_ATDrive(SUB_AprilTag at, SUB_Drivetrain dt) {
        aprilTag = at;
        drive = dt;
        addRequirements(at);
    } 
    //  in inches
    

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // We are using the Y value, because the Y value will change as you get closer or further
        // We got a pretty accurate value for what the y value should be in order for us to get as close as we can
        double distance = aprilTag.getDistance();

        // We check if we can even find one to begin with, or it will be impossible to turn to something tha tdoesn't exist
        if(aprilTag.getTv()){
            // We found that -3.6 is the correct number for Y, so as long as we don't reach it, we will drive forward
            if(distance > 12){
                drive.setMotorsArcade( 0.4, 0.0);
                SmartDashboard.putNumber("ATDISTANCE", distance);
            }
                
            
        }

        SmartDashboard.putBoolean("LimeDriveStart", true);
        System.out.println("LimeDrive: True");
    }
    public boolean isFinished(){
        //If we reach the place, then we stop
        
        if (aprilTag.getDistance() <= 12){
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