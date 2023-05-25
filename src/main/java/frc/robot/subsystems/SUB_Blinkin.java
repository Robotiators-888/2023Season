/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.StateManager;

public class SUB_Blinkin extends SubsystemBase {

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private static Spark m_blinkin = null;
  //private StateManager stateManager = RobotContainer.stateManager;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public SUB_Blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void rainbow() {
    set(0.77);
  }
  public void solidRed(){
    set(0.17);
  }
  
  public void solidViolet(){
    set(0.91);
  }
  public void solidOrange() {
    set(0.65);
  }
  public void off(){
    set(0);
  }
  public void allianceColor() {
    boolean isRed = (DriverStation.getAlliance() == Alliance.Red);
    if (isRed){
      set(-0.01);
      
    } else {
      set(0.87);
    }
  }


public Command solidVioletCommand(){
  return new InstantCommand(
    ()->{solidViolet();});
}
public Command solidRedCommand(){
  return new InstantCommand(
()-> {solidRed();});
}
public Command solidOrangeCommand(){
  return new InstantCommand(
()-> {solidOrange();});
}
public Command allianceColorCommand(){
  return new InstantCommand(
()-> {allianceColor();}); 
}



  @Override
  public void periodic(){
    //System.out.println("LED: " + m_blinkin.get());

    

  }

}
