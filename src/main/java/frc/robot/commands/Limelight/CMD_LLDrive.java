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
        addRequirements(dt);
    } 
    //  in inches
    

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // Gets distance from limelight
        double distance = limelight.getDistance();

        // We check if we can even find one to begin with, or it will be impossible to turn to something tha tdoesn't exist
        if(limelight.getTv()){
            // If we aren't the correct distance away, we will drive
            if (distance > 24.5) {
                drive.driveArcadeSquared( 0.4, 0.0);
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