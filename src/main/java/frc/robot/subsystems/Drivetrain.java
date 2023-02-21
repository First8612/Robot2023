package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
    private final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */
    private final double kWheelCircumference = Units.inchesToMeters(6 * Math.PI);
    private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(1);
    private static WPI_TalonFX m_leftFollower = new WPI_TalonFX(2);
    private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(3);
    private static WPI_TalonFX m_rightFollower = new WPI_TalonFX(4);
    private static WPI_TalonFX[] m_motors = { m_leftMotor, m_leftFollower, m_rightMotor, m_rightFollower };
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor, m_rightFollower);

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0, 0); // TODO: Pull these from SysId

    private final double p = 0, i = 0, d = 0; // TODO: tune these
    private final PIDController m_leftPIDController = new PIDController(p, i, d);
    private final PIDController m_rightPIDController = new PIDController(p, i, d);
  
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
        m_robotDrive.arcadeDrive(speed, -rotation * 0.75);
    }

    public void setMaxSpeed(double maxOutput) {
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void resetMaxSpeed() {
        m_robotDrive.setMaxOutput(m_defaultSpeed);
    }

    public double getLeftDistanceMeters() {
        return m_leftMotor.getSelectedSensorPosition()
                * kUnitsPerRevolution
                * kWheelCircumference;
    }

    public double getRightDistanceMeters() {
        return m_rightMotor.getSelectedSensorPosition()
                * kUnitsPerRevolution
                * kWheelCircumference;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            m_leftMotor.getSelectedSensorVelocity()
                * kUnitsPerRevolution
                * kWheelCircumference,
            m_rightMotor.getSelectedSensorVelocity()
                * kUnitsPerRevolution
                * kWheelCircumference
        );
      }
    
    public SimpleMotorFeedforward getFeedforward() {
        return m_feedForward;
    }

    public PIDController getLeftPIDController() {
        return m_leftPIDController;
    }

    public PIDController getRightPIDController() {
        return m_rightPIDController;
    }

    public void setOutputVolts(double left, double right) {
        var rightSetpoint = right / 12;
        var leftSetpoint = left / 12;

        SmartDashboard.putNumber("Left Motor Setpoint (in)", leftSetpoint);
        SmartDashboard.putNumber("Right Motor Setpoint (in)", rightSetpoint);

        m_leftMotor.set(leftSetpoint);
        m_rightMotor.set(rightSetpoint);

        SmartDashboard.putNumber("Left Motor Setpoint (out)", m_leftMotor.get());
        SmartDashboard.putNumber("Right Motor Setpoint (out)", m_rightMotor.get());
    }
}
