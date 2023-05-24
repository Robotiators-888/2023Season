package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.StateManager;

public class SUB_AprilTag extends SubsystemBase{
    NetworkTable table;
    final SUB_Drivetrain drive = RobotContainer.drivetrain;
    final SUB_Tower tower = RobotContainer.tower;
    final StateManager stateManager = RobotContainer.stateManager;
    final PIDController turnPID = new PIDController(0.02,0,0); 
    
    public SUB_AprilTag() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        turnPID.enableContinuousInput(-180, 180);
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
        while(this.getDistance() > 45){
            if(this.getTv()){
                // If we are more than a inch away, we will drive for
                    double movespeed;
                        if (this.getDistance() > 40){
                            movespeed =  0.45;
                        }else if(this.getX() > 35){
                            movespeed = 0.35;
                        }else{
                            movespeed = 0.3;
                        }
                    drive.driveArcade(movespeed, 0);
                    System.out.println(movespeed);
                    System.out.println(this.getDistance());
                    SmartDashboard.putNumber("ATDISTANCE", this.getDistance());
                    Logger.getInstance().recordOutput("AprilTag/Distance", this.getDistance());
                    SmartDashboard.putNumber("ATmovespeed", movespeed);
            }
        } 
            
    }
    public void aprilDrive2(){
        while(this.getDistance() > 25){
            if(this.getTv()){
                // If we are more than a inch away, we will drive for
                    double movespeed;
                        if (this.getDistance() > 30){
                            movespeed =  0.275;
                        }else{
                            movespeed = 0.25;
                        }
                    drive.driveArcade(movespeed, 0);
                    System.out.println(movespeed);
                    System.out.println(this.getDistance());
                    SmartDashboard.putNumber("ATDISTANCE", this.getDistance());
                    Logger.getInstance().recordOutput("AprilTag/Distance", this.getDistance());
                    SmartDashboard.putNumber("ATmovespeed", movespeed);
            }
        }
        } 
    public void aprilAlign(){
        // while(Math.abs(this.getX()) >= 1 && this.getTv()){
        
        //     if (this.getTv()){
        //         if (this.getX() > 1) { // turn left
        //             double turnSpeed;
        //             if (this.getX() > 10){
        //                 turnSpeed =  -0.32;
        //             }else if(this.getX() > 4.5){
        //                 turnSpeed = -0.255;
        //             }else{
        //                 turnSpeed = -0.25;
        //             }
        //             drive.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
        //             SmartDashboard.putNumber("ATturnspeed: ", turnSpeed);
        //             SmartDashboard.putBoolean("aligning", true);
        //             Logger.getInstance().recordOutput("AprilTag/TurnSpeed", turnSpeed);
        //             Logger.getInstance().recordOutput("AprilTag/AlignBool", true);
        //             System.out.println(turnSpeed);
        //             System.out.println(this.getX());
        //         } else if (this.getX() < -1){ // turn right
        //             double turnSpeed = 0;
        //             if (this.getX() < -10){
        //                 turnSpeed =  0.32;
        //             }else if(this.getX() < -4.5){
        //                 turnSpeed = 0.255;
        //             }else{
        //                 turnSpeed = 0.25;
        //             }
        //             drive.driveArcade(0.0, turnSpeed); // If we are further away, we will turn faster
        //             SmartDashboard.putNumber("ATturnspeed: ", turnSpeed);
        //             SmartDashboard.putBoolean("aligning", true);
        //             Logger.getInstance().recordOutput("AprilTag/TurnSpeed", turnSpeed);
        //             Logger.getInstance().recordOutput("AprilTag/AlignBool", true);
        //             System.out.println(turnSpeed);
        //             System.out.println(this.getX());
        //         } else {
        //             System.out.println(0);
        //             System.out.println(this.getX());
        //             drive.setBrakeMode(true);
        //         }
        //         SmartDashboard.putBoolean("limeAlign", true);
        //     } else {
        //         drive.setBrakeMode(true);
        //         break;
        //     }  
        //  }  
        // drive.setBrakeMode(true);
        drive.turnToTheta(getX());
    }

    // public Command score(){
    //     return new SequentialCommandGroup(
    //         blinkin.rainbowCommand(),
    //         new InstantCommand(() -> {this.aprilAlign();}, this).withTimeout(3).andThen(() -> {drive.setBrakeMode(true);},drive),
    //         new InstantCommand(() -> {this.aprilDrive();}, this).withTimeout(3).andThen(() -> {drive.setBrakeMode(true);},drive),
    //         new InstantCommand(() -> {this.aprilAlign();}, this).withTimeout(3).andThen(() -> {drive.setBrakeMode(true);},drive),
    //                 new InstantCommand(() -> tower.setTargetPosition(stateManager.kScoringPosition(), tower)),
    //                 new WaitCommand(2),
    //                 new InstantCommand(()-> stateManager.outtakeRoller()),
    //             new SequentialCommandGroup(
    //                new WaitCommand(1),
    //                new InstantCommand(()->stateManager.stopRoller()),
    //                 new InstantCommand(()-> tower.setTargetPosition(Constants.Arm.kHomePosition, tower)))
    //                 );
    // }
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

