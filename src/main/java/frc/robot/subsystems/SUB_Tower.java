package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class SUB_Tower extends SubsystemBase {
    public CANSparkMax towerMotor = new CANSparkMax(Constants.TOWER_SPARKMAX_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public RelativeEncoder m_encoder;

    public void setLimits(){
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        towerMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 128);
        towerMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    }

    public void towerMove(double speed) {
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
