package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.PIDGains;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class SUB_Tower extends SubsystemBase {
    //set motors

    private CANSparkMax armMotor = new CANSparkMax(Constants.TOWER_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private TrapezoidProfile m_profile;
    private Timer m_timer;

    //declare encoders
    private SparkMaxPIDController m_controller;
    private double m_setpoint;
    private RelativeEncoder m_encoder;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;
    

    DataLog log = DataLogManager.getLog();
    DoubleLogEntry towerMotorOutput = new DoubleLogEntry(log, "/tower/motorOutput");
    DoubleLogEntry towerEncoderRotations = new DoubleLogEntry(log, "/tower/encoderRotations");
    DoubleLogEntry towerDegreesRotations = new DoubleLogEntry(log, "/tower/encoderDegreesRotations");

    //declares and sets PID, setpoint is arbitrary
    PIDController pid = new PIDController(Constants.PID_kP, Constants.PID_kI, Constants.PID_kD);
    double setpoint = 42.0;
    //Counteract Gravity on Arm, Currently lbsArm is arbitrary (For kG of FF)
    double lbsArm = 30.0;
    double gravitional_force_in_Kg = (lbsArm*4.44822162)/9.8;
    // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
    //ArmFeedforward feedforward = new ArmFeedforward(Constants.FF_kS, Constants.FF_kG, Constants.FF_kV, Constants.FF_kA);
    
    public SUB_Tower(){
        //resetEncoder();
        setLimits();
        armMotor.setIdleMode(IdleMode.kBrake); // sets brake mode 
        armMotor.setOpenLoopRampRate(0.6); // motor takes 0.6 secs to reach desired power
        armMotor.restoreFactoryDefaults();
        armMotor.setInverted(false);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.kSoftLimitForward);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.kSoftLimitReverse);
        
        m_encoder = armMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        m_encoder.setPositionConversionFactor(Constants.Arm.kArmGearRatio);
        m_encoder.setVelocityConversionFactor(Constants.Arm.kArmGearRatio);

        m_controller = armMotor.getPIDController();
        PIDGains.setSparkMaxGains(m_controller, Constants.Arm.kArmPositionGains);

        m_setpoint = Constants.Arm.kHomePosition;

        m_timer = new Timer();
    m_timer.start();
    m_timer.reset();

    updateMotionProfile();
    }

    public void setTargetPosition(double _setpoint, SUB_Tower _gripper) {
        if (_setpoint != m_setpoint) {
          m_setpoint = _setpoint;
          updateMotionProfile();
        }
      }

      private void updateMotionProfile() {
        TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
        TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
        m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, goal, state);
        m_timer.reset();
      }

      public void runAutomatic() {
        double elapsedTime = m_timer.get();
        if (m_profile.isFinished(elapsedTime)) {
          targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
        }
        else {
          targetState = m_profile.calculate(elapsedTime);
        }
    
        feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
        m_controller.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
      }
      public void runManual(double _power) {
        //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
        m_setpoint = m_encoder.getPosition();
        targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
        m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, targetState, targetState);
        //update the feedforward variable with the newly zero target velocity
        feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.Arm.kArmZeroCosineOffset, targetState.velocity);
        armMotor.set(_power + (feedforward / 12.0));
        manualValue = _power;
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
        SmartDashboard.putNumber("setpoint", m_setpoint);
        SmartDashboard.putNumber("time", m_timer.get());
        SmartDashboard.putNumber("feedfoward", feedforward);
        SmartDashboard.putNumber("manual value", manualValue);
        SmartDashboard.putNumber("encoder positiion", m_encoder.getPosition());
        SmartDashboard.putNumber("encoder velocity", m_encoder.getVelocity());
        SmartDashboard.putNumber("encoder counts/rev", m_encoder.getCountsPerRevolution());
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