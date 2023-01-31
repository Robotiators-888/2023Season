package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class manipSub extends SubsystemBase {
    public CANSparkMax towerMotor = new CANSparkMax(Constants.towerCAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public RelativeEncoder m_encoder;
    public void towerMoveUp() {
        towerMotor.set(0.5); 
    }
    public void towerMoveDown() {
        towerMotor.set(-0.5); 
    }
    public void towerStop() {
        towerMotor.setIdleMode(IdleMode.kBrake);
    }

    public getRotations(){
        m_encoder = towerMotor.getEncoder();
        return m_encoder.getPosition();
    }
}
