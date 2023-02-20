package frc.robot.commands.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CMD_LLDrive extends CommandBase {
    SUB_Limelight limelight;
    SUB_Drivetrain drive;

    public CMD_LLDrive(SUB_Limelight ll, SUB_Drivetrain dt) {
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
        double distance = limelight.getDistance();

        // We check if we can even find one to begin with, or it will be impossible to turn to something tha tdoesn't exist
        if(limelight.getTv()){
            // We found that -3.6 is the correct number for Y, so as long as we don't reach it, we will drive forward
            if (distance > 24.5) {
                drive.driveArcade( 0.4, 0.0);
                System.out.println(distance);
            }
        }

        SmartDashboard.putBoolean("LimeDriveStart", true);
        System.out.println("LimeDrive: True");
    }
    public boolean isFinished(){
        // If we reach the place, then we stop
        
        if (limelight.getDistance() <= 24.5){
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