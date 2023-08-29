package frc.robot.utils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// To Import This Class Use: LogiUtils logiUtils = new LogiUtils("Your Controller USB Port #");
/**
 * Wrapper Class For Logitech F310 Controller
 */
public class LogiUtils extends Joystick {
    private JoystickButton AButton;
    private JoystickButton BButton;
    private JoystickButton XButton;
    private JoystickButton YButton;
    private JoystickButton startButton;
    private JoystickButton backButton;
    private JoystickButton leftButton;
    private JoystickButton rightButton;
    private JoystickButton leftJoystickPress;
    private JoystickButton rightJoystickPress;


  /**
   * List of Axies and Their Corresponding Numbers 
   */
    public enum axis {
        kLEFTX(0),
        kLEFTY(1),
        kLEFTTRIIGER(2),
        kRIGHTTRIGGER(3),
        kRIGHTX(4),
        kRIGHTY(5);
        
        public final int value;
        /**
         * The Axis # And Corresponding Value
         * @param value
         */
        axis(int value) {
          this.value = value;
        }
    }
    /**
     * List of Buttons and Their Corresponding Numbers 
     */
    public enum button {
        kABUTTON(1),
        kBBUTTON(2),
        kXBUTTON(3),
        kYBUTTON(4),
        kLEFTBUMPER(5),
        kRIGHTBUMPER(6),
        kBACKBUTTON(7),
        kSTARTBUTTON(8),
        kLEFTJOYSTICKPRESS(9),
        kRIGHTJOYSTICKPRESS(10);
        
        public final int value;
        /**
         * The Button # And Corresponding Value
         * @param value
         */
        button(int value) {
            this.value = value;
        }
    }
    /**
     * The Constructor That Allows The User To Pass a Port Number to This Class
     * @param port,
     */
    public LogiUtils(int port){
        super(port);
        AButton = new JoystickButton(this, button.kABUTTON.value); 
        BButton = new JoystickButton(this, button.kBBUTTON.value);
        XButton = new JoystickButton(this, button.kXBUTTON.value);
        YButton = new JoystickButton(this, button.kYBUTTON.value);
        leftButton = new JoystickButton(this, button.kLEFTBUMPER.value);
        rightButton = new JoystickButton(this, button.kRIGHTBUMPER.value);
        startButton = new JoystickButton(this, button.kSTARTBUTTON.value);
        backButton = new JoystickButton(this, button.kBACKBUTTON.value);
        leftJoystickPress = new JoystickButton(this, button.kLEFTJOYSTICKPRESS.value);
        rightJoystickPress = new JoystickButton(this, button.kRIGHTJOYSTICKPRESS.value);
    }
    
    public JoystickButton getAButtonPressed(){
        return AButton;
    }       
    
    public JoystickButton getBButtonPressed(){
        return BButton;
    } 
    
    public JoystickButton getXButtonPressed(){
        return XButton;
    }     
          
    public JoystickButton getYButtonPressed(){
        return YButton;
    }       
          
    public JoystickButton getLeftBumperButtonPressed(){
        return leftButton;
    }   
    
    public JoystickButton getRightBumperButtonPressed(){
        return rightButton;
    }  
          
    public JoystickButton getStartButtonPressed(){
        return startButton;
    }   
       
    public JoystickButton getBackButtonPressed(){
        return backButton;
    }      
     
    public JoystickButton getLeftJoystickButtonPressed(){
        return leftJoystickPress;
    }       
     
    public JoystickButton getRightJoystickButtonPressed(){
        return rightJoystickPress;
    }  
     
    public double getLeftXAxis(){
        return getRawAxis(axis.kLEFTX.value);
    }
      
    public double getLeftYAxis(){
        return getRawAxis(axis.kLEFTY.value);
    }
    
    public double getRightXAxis(){
        return getRawAxis(axis.kRIGHTX.value);
    }
    
    public double getRightYAxis(){
        return getRawAxis(axis.kRIGHTY.value);
    }
  
    public double getLeftTriggerAxis(){
        return getRawAxis(axis.kLEFTTRIIGER.value);
    }
   
    public double getRightTriggerAxis(){
        return getRawAxis(axis.kRIGHTTRIGGER.value);
    }
}
    
