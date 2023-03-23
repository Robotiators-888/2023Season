package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.libs.RunningAverageQueue;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class SUB_Roller extends SubsystemBase {

    private CANSparkMax m_roller;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_controller;
    private RunningAverageQueue queue = new RunningAverageQueue(10);
    boolean isCone;

    public SUB_Roller() {
        m_roller = new CANSparkMax(Constants.Roller.kRollerCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_roller.restoreFactoryDefaults();
        m_roller.setInverted(false);
       // m_roller.setSmartCurrentLimit(Constants.Roller.kCurrentLimit);
        //m_roller.setSmartCurrentLimit(20, Constants.Roller.kCurrentLimit);
        m_roller.setSmartCurrentLimit(20, 80);
    
        m_encoder = m_roller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
        m_controller = m_roller.getPIDController();
        //PIDGains.setSparkMaxGains(m_controller, Constants.Roller.kPositionPIDGains);
    
        m_roller.burnFlash();

    }

    public boolean isCurrentLimit(){

        return m_roller.getOutputCurrent() > Constants.Roller.kHoldLimit;
    }

    // Rolls the roller forward, to intake the piece
    public void driveRoller(double speed) {
        m_roller.set(speed);
        //m_roller.
        Logger.getInstance().recordOutput("Desired Speed", speed);
    }

    
    public void periodic(){

        queue.insert(m_roller.getOutputCurrent());
        // if(queue.getRunningAverage() > Constants.Roller.kCurrentStall){
        //     driveRoller(Constants.Roller.kHoldSpeed);
        //   //  sm.grabGP();
        // }else if(queue.getRunningAverage() < Constants.Roller.kCurrentStall && 
        //     queue.getRunningAverage() > 1){
        //         driveRoller(Constants.Roller.kDriveSpeed);
        //       //  sm.dropGp();
        // }


        SmartDashboard.putNumber("Average Roller Current", queue.getRunningAverage());
        Logger.getInstance().recordOutput("Roller/SparkMaxCurrent", m_roller.getOutputCurrent());
        Logger.getInstance().recordOutput("Roller/Speed", m_encoder.getVelocity());
        Logger.getInstance().recordOutput("Roller/Voltage", m_roller.getBusVoltage());
        Logger.getInstance().recordOutput("Roller/Output", m_roller.getAppliedOutput());
        
    }

    //TODO Write test off bot for current sensing with JAVA Test
}
