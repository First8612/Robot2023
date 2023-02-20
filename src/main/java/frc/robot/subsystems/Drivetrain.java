package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
    public static WPI_TalonFX m_leftMotor = new WPI_TalonFX(1);
    public static WPI_TalonFX m_leftFollower = new WPI_TalonFX(2);
    public static WPI_TalonFX m_rightMotor = new WPI_TalonFX(3);
    public static WPI_TalonFX m_rightFollower = new WPI_TalonFX(4);
    public static WPI_TalonFX[] m_motors = {m_leftMotor, m_leftFollower, m_rightMotor, m_rightFollower};
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor, m_rightFollower);

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private double m_defaultSpeed = 0.1;

    public Drivetrain() {
        m_rightMotor.setInverted(false);
        m_leftMotor.setInverted(false);
        m_leftFollower.follow(m_leftMotor);
        m_rightFollower.follow(m_rightMotor);
    }
    
    public void reset() {
        resetMaxSpeed();
        for (var motor : m_motors) {
            motor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        }
    }

    public void arcadeDrive(double speed, double rotation) {
        m_robotDrive.arcadeDrive(speed, rotation * 0.75);
    } 
    
    public void setMaxSpeed(double maxOutput) {
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void resetMaxSpeed() {
        m_robotDrive.setMaxOutput(m_defaultSpeed);
    }
}
