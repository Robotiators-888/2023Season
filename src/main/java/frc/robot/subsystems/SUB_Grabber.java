package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SUB_Grabber extends SubsystemBase {
    //set motors
    public CANSparkMax grabberMotor = new CANSparkMax(Constants.GRIPPER_SPARKMAX_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    //declare encoders
    //public SparkMaxAbsoluteEncoder.Type m_encoder = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
    public RelativeEncoder m_encoder = grabberMotor.getEncoder();
    //public AbsoluteEncoder throughBoreEncoder = towerMotor.getAbsoluteEncoder(m_encoder);

   
    // true = cone
    // false = cube
    boolean gamepiece = false;

    SUB_Grabber(){
        m_encoder.setPosition(0);
        setLimits();
    }

    public void setLimits(){
        //set soft limits and current limits for how far the manip can move
        grabberMotor.setSmartCurrentLimit(10, 20);

        grabberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        grabberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        grabberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 25);
        grabberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    }

    public void grabberMove(double speed) {
        grabberMotor.set(speed);
        //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
        //towerMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getRotations(){
        //gets position
        return m_encoder.getPosition();
    }

    public void toggleGamepiece(){
        gamepiece = !gamepiece;
    }
}
