package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

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
        double h1 = 24.25;
        double h2 = 18;
        // was -3.47935054
        double a1 = Math.toRadians(0); //mounting angle, radians
        double a2 = Math.toRadians(this.getY());

        if(this.getTv()){
            double distance = (h2 - h1)/Math.tan(a1 + a2);
            return distance;
        }else{
            return 0.0;
        }
        
    }

    // Gets the angle offset on the x plane to know how far to align
    public double getX() {
        if(this.getTv()){
            return table.getEntry("tx").getDouble(0.0);
        }else{
            return 0.0;
        }
        
    }

    // Gets the angle offset on the y plane to know how close you have to get
    public double getY() {
        if(this.getTv()){
            return table.getEntry("ty").getDouble(0.0);
        }else{
            return 0.0;
        }
    }

    public void aprilDrive(){
        if(this.getTv()){
            // If we are more than a inch away, we will drive for
            if(this.getDistance() > 6){
                double movespeed = Math.min(Math.max(this.getDistance() * -0.03, -0.5),-0.265);
                drive.driveArcade(movespeed, 0);
                SmartDashboard.putNumber("ATDISTANCE", this.getDistance());
                Logger.getInstance().recordOutput("AprilTag/Distance", this.getDistance());
                SmartDashboard.putNumber("movespeed", movespeed);
            }
        }
    }

    public void aprilAlign(){
        if (this.getTv()){
            if (this.getX() > 0.009) { // turn left
                double turnSpeed = -Math.min(Math.max(this.getX() * -0.03, -0.5),-0.265);
                drive.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
                Logger.getInstance().recordOutput("AprilTag/TurnSpeed", turnSpeed);
                Logger.getInstance().recordOutput("AprilTag/AlignBool", true);
                System.out.println(turnSpeed);
            } else if (this.getX() < -0.009){ // turn right
                double turnSpeed = -Math.max(Math.min(this.getX() * -0.03, 0.5),0.265);
                drive.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
                SmartDashboard.putNumber("turnspeed: ", turnSpeed);
                SmartDashboard.putBoolean("aligning", true);
                Logger.getInstance().recordOutput("AprilTag/TurnSpeed", turnSpeed);
                Logger.getInstance().recordOutput("AprilTag/AlignBool", true);
                System.out.println(turnSpeed);
            }
            else{
                SmartDashboard.putBoolean("aligning", false);
                Logger.getInstance().recordOutput("AprilTag/AlignBool", false);
                drive.setBrakeMode(true);
            }
            SmartDashboard.putBoolean("limeAlign", true);
        }
    }

    public Command score(){
        return new SequentialCommandGroup(
        new RunCommand(() -> {this.aprilAlign();}, this).until(() -> (Math.abs(this.getX()) <= 0.05 || !this.getTv())),
        new InstantCommand(() -> {SmartDashboard.putNumber("turnspeed", 0);}, this),
        new RunCommand(() -> {this.aprilDrive();}, this).until(() -> (this.getDistance() <= 6 || !this.getTv())),
        new InstantCommand(() -> {SmartDashboard.putNumber("movespeed", 0);}, this),
        new RunCommand(() -> {this.aprilAlign();}, this).until(() -> (Math.abs(this.getX()) <= 0.05 || !this.getTv())),
        new InstantCommand(() -> {SmartDashboard.putNumber("turnspeed", 0);}, this));
    }

    public void periodic() {
        //Sets all the method calls to the SmartDashboard
        SmartDashboard.putNumber("ATY", this.getY());
        SmartDashboard.putBoolean("ATTarget", this.getTv());
        SmartDashboard.putNumber("ATX", this.getX());
        SmartDashboard.putNumber("ATDistance", this.getDistance());
        SmartDashboard.putNumber("a1", Math.toRadians(0));
        SmartDashboard.putNumber("a2", Math.toRadians(this.getY()));
        SmartDashboard.putBoolean("IS IT THERE", this.getTv());

        Logger.getInstance().recordOutput("AprilTag/ATY", this.getY());
        Logger.getInstance().recordOutput("AprilTag/ATTarget", this.getTv());
        Logger.getInstance().recordOutput("AprilTag/ATX", this.getX());
        Logger.getInstance().recordOutput("AprilTag/ATDistance", this.getDistance());
        Logger.getInstance().recordOutput("AprilTag/a1", Math.toRadians(0));
        Logger.getInstance().recordOutput("AprilTag/a2", Math.toRadians(this.getY()));
    }
}

