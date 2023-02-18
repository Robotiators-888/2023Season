package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;

public class SUB_Tower extends SubsystemBase {
    //set motors
    public CANSparkMax armMotor = new CANSparkMax(Constants.TOWER_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    //declare encoders
    public RelativeEncoder m_encoder = armMotor.getEncoder();

    DataLog log = DataLogManager.getLog();
    DoubleLogEntry towerMotorOutput = new DoubleLogEntry(log, "/tower/motorOutput");
    DoubleLogEntry towerEncoderRotations = new DoubleLogEntry(log, "/tower/encoderRotations");
    DoubleLogEntry towerDegreesRotations = new DoubleLogEntry(log, "/tower/encoderDegreesRotations");

    //declares and sets PID, setpoint is arbitrary
    PIDController pid = new PIDController(Constants.PID_kP, Constants.PID_kI, Constants.PID_kD);
    double setpoint = 42.0;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    double lbsArm = 30;
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;
    // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
    //ArmFeedforward feedforward = new ArmFeedforward(Constants.FF_kS, Constants.FF_kG, Constants.FF_kV, Constants.FF_kA);
    
    public SUB_Tower(){
        resetEncoder();
        setLimits();
        armMotor.setIdleMode(IdleMode.kBrake); // sets brake mode 
        armMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
    }

    // set power and position limits
    public void setLimits(){
        //set soft limits and current limits for how far the manip can move
        armMotor.setSmartCurrentLimit(40, 20);
        
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        // stops motor at 130 encoder clicks, (touching the ground)
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 130);
        // stops motor at 0 encoder clicks when reversing, (touching the robot)
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    }

    // not used
    public void armMove(double speed) {
        //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
        armMotor.set(speed);
        //System.out.println("feedforward: "+feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
    }

    

    public void periodic() {
        towerMotorOutput.append(armMotor.get());
        towerEncoderRotations.append(getRotations());
        towerDegreesRotations.append(calculateDegreesRotation());
        SmartDashboard.putNumber("Arm Current Rotations: ", getRotations());
        SmartDashboard.putNumber("Arm degreesRotation", calculateDegreesRotation());
        SmartDashboard.putNumber("Arm degreesRotationCosineAngle",Constants.FF_kG*Math.cos(Math.toRadians(calculateDegreesRotation())) );
        SmartDashboard.putNumber("Arm TowerMotorCurrent", armMotor.getOutputCurrent());
    }

    // balances the arm using feedforward, then adds on volts to move the arm.
    public void armMoveVoltage(double volts) {
        //towerMotor.set(pid.calculate(getRotations(), setpoint) + feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
        armMotor.setVoltage(volts+getAutoBalanceVolts());// sets voltage of arm -12 to 12 volts
        SmartDashboard.putNumber("TowerMotorVolts", volts+getAutoBalanceVolts());
        //System.out.println("feedforward: "+feedforward.calculate(Constants.FF_Velocity, Constants.FF_Accel));
    }

    // calculates volts to counteract gravity based on position of the arm
    public double getAutoBalanceVolts(){
        // Math.cos(theta) as more downward force increases near 0,180 degrees
       return (Constants.FF_kG*Math.cos(Math.toRadians(calculateDegreesRotation())));
    }

    // converts encoder clicks of arm into arms rotation in degrees
    public double calculateDegreesRotation(){
        double horizontalDegrees = 31.75;
        double encoderClicksToDegrees = 1.875;
        return (encoderClicksToDegrees*getRotations())-horizontalDegrees;
    }

    // get arm encoder clicks
    public double getRotations(){
        //gets position
        return m_encoder.getPosition();
    }

    public void resetEncoder(){
        m_encoder.setPosition(0);
    }
}