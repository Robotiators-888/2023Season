package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.Constants;


public class AutoBalance extends CommandBase{
    private SUB_Drivetrain m_driveTrain;

    private double error;
    private double currentAngle;
    private double lastAngle = 0;
    private double drivePower;
    private long balanceTime = 0;
    private double ForwardMult = 1.5; // must have its own max speed
    private double maxSpeed = 0.5;
    
    // keep tuning or add more sensors?
    // try stressballbot for less heavy

    public AutoBalance(SUB_Drivetrain drivetrain) {
      this.m_driveTrain = drivetrain;
      addRequirements(drivetrain);
    }
  
    public long getMiliSeconds(){
      return System.currentTimeMillis();
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      balanceTime = getMiliSeconds();
      lastAngle = m_driveTrain.getRoll();

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
       // sets angle to roll: angle the balence beam can rotate.
      this.currentAngle = m_driveTrain.getRoll();

      // calculates diference in angle since last tick
      // dif in angle will increase when first driving on the beam and when the beam rotates.
      double diferenceInAngle = this.currentAngle - this.lastAngle;

     
      // calculate error based on angle and target angle... 0
      error = 0 - currentAngle;
      // calculate motor speed, proportionally based on angle error, then multiple by a constant KP
      drivePower = -Math.min(Constants.Autonomous.BALANCE_KP * error, 1);
      

      // detects if beam flipping down, then permanently decreases speeds
    //was -0.7
    if (-1.2 > diferenceInAngle || currentAngle < 0){
      ForwardMult = 1;
      maxSpeed = 0.43;
     
    }

    // reverse if beam swinging fast
    // if (Math.abs(diferenceInAngle) > 0.7){
    //   SmartDashboard.putBoolean("Balencing reversing", true);
    //   drivePower *= -1.5;
    // }
    // else{
    //   SmartDashboard.putBoolean("Balencing reversing",false);

    // }

      // add difInAngle to drivePower
      // decreases speed if angle changing quickly
      drivePower += diferenceInAngle*Constants.Autonomous.BALANCE_KD;

      // caps speed at maxSpeed, Maxspeed decreases after first beam flip
      if (Math.abs(drivePower) > maxSpeed) {
        drivePower = Math.copySign(maxSpeed, drivePower);
      }
      // sets speed to 20% if its lower, (robot will not climb beam lower than 20%)
      else if ((Math.abs(drivePower) < 0.2)){
        drivePower = Math.copySign(0.2, drivePower);
      }

      //if balenced for 0.15 seconds, then stop driving
      if ((getMiliSeconds()-balanceTime) > 150 ){ // && Math.abs(diferenceInAngle) > x
        SmartDashboard.putBoolean("balancing", true);
        m_driveTrain.setMotorsArcade(0, 0);
      }
      else{// otherwise drive!
        SmartDashboard.putBoolean("balancing", false);
        m_driveTrain.setMotorsArcade(drivePower*ForwardMult, 0);

      }

      //System.out.println("drivePower*FM: "+(drivePower*ForwardMult)+"angle: "+currentAngle+" ForwardMult:"+ForwardMult+" difInAngle: "+diferenceInAngle+" maxSpeed: "+maxSpeed);
      SmartDashboard.putNumber("drivePower*FM", (drivePower*ForwardMult));
      SmartDashboard.putNumber("roll angle",currentAngle);
      SmartDashboard.putNumber("ForwardMult",ForwardMult);
      SmartDashboard.putNumber("difInAngle", diferenceInAngle);
      SmartDashboard.putNumber("AutoBalance maxSpeed", maxSpeed);
      
      
      this.lastAngle = currentAngle;
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_driveTrain.setBrakeMode(true);
    }

    @Override
    public boolean isFinished() {
      SmartDashboard.putNumber("balanceTime Milisecs", (getMiliSeconds()-balanceTime));

      // not balenced, reset timer
      if (!(currentAngle > -5 && currentAngle < 5)){
        balanceTime = getMiliSeconds();
      }
      // if balenced for 2 secs, lock motors and finish
      return ((getMiliSeconds()-balanceTime) > 2000);

    }
}
