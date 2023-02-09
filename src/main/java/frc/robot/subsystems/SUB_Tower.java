package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

public class SUB_Tower extends SubsystemBase {
    //set motors
    public CANSparkMax towerMotor = new CANSparkMax(Constants.TOWER_SPARKMAX_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    //declare encoders
    public SparkMaxAbsoluteEncoder.Type m_encoder = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
    public AbsoluteEncoder throughBoreEncoder = towerMotor.getAbsoluteEncoder(m_encoder);

    //declares and sets PID, setpoint is arbitrary
    PIDController pid = new PIDController(Constants.PID_kP, Constants.PID_kI, Constants.PID_kD);
    double setpoint = 42.0;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    double lbsArm = 30;
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;
    // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
    ArmFeedforward feedforward = new ArmFeedforward(Constants.FF_kS, Constants.FF_kG, Constants.FF_kV, Constants.FF_kA);
    

    public void setLimits(){
        //set soft limits and current limits for how far the manip can move
        towerMotor.setSmartCurrentLimit(0, 20);
        
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 25);
        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    }

    public void towerMove(double speed) {
        //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
        towerMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getRotations(){
        //gets position
        return throughBoreEncoder.getPosition();
    }
}
