package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeDrive extends CommandBase {
    Limelight limelight;
    Drivetrain drive;

    public LimeDrive(Limelight ll, Drivetrain dt) {
        limelight = ll;
        drive = dt;
        addRequirements(ll,dt);
    } 
    //  in inches
    

    @Override
    public void initialize() {
        //limelight.setLed(3);
    }
    
    @Override
    public void execute() {
        double distance = limelight.getY();
        if(limelight.getTv()){
            if (distance > -14) {
                drive.setMotorsArcade(0.5, 0.0);
                System.out.println(distance);
            }
        }
        SmartDashboard.putBoolean("LimeHasTarget", limelight.getTv());
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("Distance", distance);
    }
    public boolean isFinished(){
        if(limelight.getY() <= -14){
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