package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SUB_Limelight extends SubsystemBase{
    NetworkTable table;

    public SUB_Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
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
    public double getTy() {
        return table.getEntry("ty").getDouble(0);
    }
    // Finds distance from robot to target and returns distance
    public double getDistance() {
        double h1 = 35.625;
        double h2 = 23.0625;
        double a1 = 0;
        double a2 = Math.toRadians(Math.abs(this.getTy()));

        return Math.abs((double) ((h2 - h1) / (Math.tan(a1 + a2))));
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
}
