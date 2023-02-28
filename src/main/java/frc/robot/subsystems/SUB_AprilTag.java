package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SUB_AprilTag extends SubsystemBase{
    NetworkTable table;
    final SUB_Drivetrain drive = RobotContainer.drivetrain;
    public SUB_AprilTag() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    public void switchapipeline(int pipelineNumber){
      // Allows the pipeline to switch
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
        double h2 = 18;
        // was -3.47935054
        double a1 = Math.toRadians(0); //mounting angle, radians
        double a2 = Math.toRadians(this.getY());

        double distance = (h2 - h1)/Math.tan(a1 + a2);

        return distance;
    }

    // Gets the angle offset on the x plane to know how far to align
    public double getX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    // Gets the angle offset on the y plane to know how close you have to get
    public double getY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public void aprilDrive(){
        if(this.getTv()){
            // If we are more than a feet away, we will drive for
            if(this.getDistance() > 12){
                drive.driveArcadeSquared( 0.4, 0.0);
                SmartDashboard.putNumber("ATDISTANCE", this.getDistance());
            }
        }
    }

    public void aprilAlign(){
        if (this.getTv()){
            if (this.getX() > 0.009) { // turn left
                double turnSpeed = -Math.min(Math.max(this.getX() * -0.03, -0.5),-0.265);
                drive.driveArcadeSquared(0.0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
            } else if (this.getX() < -0.009){ // turn right
                double turnSpeed = -Math.max(Math.min(this.getX() * -0.03, 0.5),0.265);
                drive.driveArcadeSquared(0.0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("Limelight turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
            }

            SmartDashboard.putBoolean("limeAlign", true);
        }
    }

    public void periodic() {
        //Sets all the method calls to the SmartDashboard
        SmartDashboard.putNumber("ATY", this.getY());
        SmartDashboard.putBoolean("ATTarget", this.getTv());
        SmartDashboard.putNumber("ATX", this.getX());
        SmartDashboard.putNumber("ATDistance", this.getDistance());
        SmartDashboard.putNumber("a1", Math.toRadians(0));
        SmartDashboard.putNumber("a2", Math.toRadians(this.getY()));

    }
}

