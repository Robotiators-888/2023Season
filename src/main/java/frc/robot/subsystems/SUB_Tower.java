package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class SUB_Tower extends SubsystemBase {
    public CANSparkMax towerMotor = new CANSparkMax(Constants.TOWER_SPARKMAX_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public RelativeEncoder m_encoder;
    
    //setpoint is arbitrary
    PIDController pid = new PIDController(Constants.PID_kP, Constants.PID_kI, Constants.PID_kD);
    double setpoint = 42.0;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    double lbsArm = 30;
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;
    // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
    ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.FF_kS, Constants.FF_kG, Constants.FF_kV, Constants.FF_kA);
    

    public void setLimits(){
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 128);
        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    }

    public void towerMove(double speed) {
        //towerMotor.set(pid.calculate(getRotations(), setpoint));
        towerMotor.set(speed); 
    }   

    public void towerStop() {
        towerMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getRotations(){
        m_encoder = towerMotor.getEncoder();
        return m_encoder.getPosition();
    }
}
