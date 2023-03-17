package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import frc.libs.PIDGains;
import frc.libs.RunningAverageQueue;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;
import frc.robot.StateManager;

public class SUB_Roller extends SubsystemBase {

    private StateManager sm = new StateManager();
    private CANSparkMax m_roller;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_controller;
    private RunningAverageQueue queue = new RunningAverageQueue(10);

    public SUB_Roller() {
        m_roller = new CANSparkMax(Constants.Roller.kRollerCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_roller.restoreFactoryDefaults();
        m_roller.setInverted(false);
        m_roller.setSmartCurrentLimit(Constants.Roller.kCurrentLimit);
        m_roller.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_roller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_roller.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Roller.kSoftLimitForward);
        m_roller.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Roller.kSoftLimitReverse);
    
        m_encoder = m_roller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
        m_controller = m_roller.getPIDController();
        PIDGains.setSparkMaxGains(m_controller, Constants.Roller.kPositionPIDGains);
    
        m_roller.burnFlash();

    }

    public boolean isCurrentLimit(){

        return m_roller.getOutputCurrent() > Constants.Roller.kHoldLimit;
    }

    // Rolls the roller forward, to intake the piece
    public void driveRoller(double speed) {
        m_roller.set(speed);
    }

    
    public void periodic(){

        queue.insert(m_roller.getOutputCurrent());
        if((queue.getRunningAverage() < 1 || queue.getRunningAverage() > -1) || 
            (RobotContainer.tower.setpoint == Constants.Arm.kHomePosition)){
                driveRoller(0.0);
       
         }else if(queue.getRunningAverage() > Constants.Roller.kCurrentStall){
            driveRoller(Constants.Roller.kHoldSpeed);
            sm.grabGP();
        }else if(queue.getRunningAverage() < Constants.Roller.kCurrentStall && 
            queue.getRunningAverage() > 1){
                driveRoller(Constants.Roller.kDriveSpeed);
                sm.dropGp();
        }

        Logger.getInstance().recordOutput("Roller/Speed", m_encoder.getVelocity());
        Logger.getInstance().recordOutput("Roller/Voltage", m_roller.getBusVoltage());
        Logger.getInstance().recordOutput("Roller/Output", m_roller.getAppliedOutput());
    }

    
}
