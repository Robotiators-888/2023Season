package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;


public class SUB_Limelight extends SubsystemBase{
    NetworkTable table;
    final SUB_Drivetrain drivetrain = RobotContainer.drivetrain;


    public SUB_Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        
    }
    
    public void switchapipeline(int pipelineNumber){
        // Switches the pipeline
        NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
        pipelineEntry.setNumber(pipelineNumber);
    }
    /* Whether the limelight has any valid targets
   * 
   * @return boolean true if target is found false if not, return false
   */
    public boolean getTv() {
        if (table.getEntry("tv").getDouble(0) == 1) {
            return true;
        } 

        else {
            return false;
        }
    }
    /**
   * Crosshair offset to target y-value
   * 
   * @return double of offset of target y-value
   */

    // Finds distance from robot to target and returns distance
    public double getDistance() {
        double h1 = 26.5;
        double h2 = 24.25;
        // was -3.47935054
        double a1 = Math.toRadians(0); //mounting angle, radians
        double a2 = Math.toRadians(this.getY());

        double distance = Math.abs((h2 - h1)/Math.tan(a1 + a2));

        return distance;
    }
    // turns on limelight(mainly used for) (force on)
    public void setLed(int value) {
        table.getEntry("ledMode").setNumber(value);
    }

    // Gets the angle offset on the x plane to know how far to align
    public double getX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    // Gets the angle offset on the y plane to know how close you have to get
    public double getY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    public void limelightAlign(){
        if (this.getTv()){
            if (getX() > 0.009) { // turn left
                double turnSpeed = -Math.min(Math.max(getX() * -0.03, -0.5),-0.265);
                drivetrain.driveArcadeSquared(0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
                Logger.getInstance().recordOutput("LL/turnSpeed", turnSpeed);
                Logger.getInstance().recordOutput("LL/Aligning", true);
            } else if (getX() < -0.009){ // turn right
                double turnSpeed = -Math.max(Math.min(getX() * -0.03, 0.5),0.265);
                drivetrain.driveArcadeSquared(0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
                Logger.getInstance().recordOutput("LL/turnSpeed", turnSpeed);
                Logger.getInstance().recordOutput("LL/Aligning", true);
            }else{
                drivetrain.setBrakeMode(true);
            }
    
        }
    }
    
    public void limelightDrive(){
        if (this.getTv()){
            if (this.getDistance() > 24.5) {
                drivetrain.driveArcadeSquared( 0.4, 0.0);
                System.out.println(this.getDistance());
            }
        }
    }
    public void periodic() {
        //Sets all the method calls to the SmartDashboard
        SmartDashboard.putNumber("LimelightY", this.getY());
        SmartDashboard.putBoolean("LimeHasTarget", this.getTv());
        SmartDashboard.putNumber("LimelightX", this.getX());
        SmartDashboard.putNumber("LIMEDistance", this.getDistance());
        SmartDashboard.putNumber("a1", Math.toRadians(0));
        SmartDashboard.putNumber("a2", Math.toRadians(this.getY()));

        Logger.getInstance().recordOutput("LL/Y", this.getY());
        Logger.getInstance().recordOutput("LL/HasTarget", this.getTv());
        Logger.getInstance().recordOutput("LL/X", this.getX());
        Logger.getInstance().recordOutput("LL/Distance", this.getDistance());
        Logger.getInstance().recordOutput("LL/a1", Math.toRadians(0));
        Logger.getInstance().recordOutput("LL/a2", Math.toRadians(this.getY()));

    }
}

