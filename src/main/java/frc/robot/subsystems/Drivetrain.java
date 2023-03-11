package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
    private final double kUnitsPerRevolution = 2048; /* this is constant for Talon FX */
    private final double kWheelCircumference = Units.inchesToMeters(6.25 * Math.PI);
    private final double kGearRatio = 8.8;
    private final double kSensorToMetersRatio = (1.0 / kUnitsPerRevolution) * kWheelCircumference * (1.0 / kGearRatio);
    private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(1);
    private static WPI_TalonFX m_leftFollower = new WPI_TalonFX(2);
    private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(3);
    private static WPI_TalonFX m_rightFollower = new WPI_TalonFX(4);
    private static WPI_TalonFX[] m_motors = { m_leftMotor, m_leftFollower, m_rightMotor, m_rightFollower };
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor, m_rightFollower);
    //private final SlewRateLimiter filterForwardBack = new SlewRateLimiter(10);
    //private final SlewRateLimiter filterRotation = new SlewRateLimiter(0.5);

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.086571, 1.9726, 0.18671);

    private final double p = 0.23126, i = 0, d = 0;
    private final PIDController m_leftPIDController = new PIDController(p, i, d);
    private final PIDController m_rightPIDController = new PIDController(p, i, d);
  
    private double m_defaultSpeed = 1;

    public Drivetrain() {
        m_rightMotor.setInverted(false);
        m_leftMotor.setInverted(true);
        m_leftFollower.setInverted(true);
        m_leftFollower.follow(m_leftMotor);
        m_rightFollower.follow(m_rightMotor);

        SmartDashboard.putData("Drivebase/Motors/RightLeader", m_rightMotor);
        SmartDashboard.putData("Drivebase/Motors/RightFollower", m_rightFollower);
        SmartDashboard.putData("Drivebase/Motors/LeftLeader", m_leftMotor);
        SmartDashboard.putData("Drivebase/Motors/LeftFollower", m_leftFollower);

        for (var motor : m_motors) {
            motor.configOpenloopRamp(0.5);
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        collectExtraMotorTelemetry("LeftLeader", m_leftMotor);
        collectExtraMotorTelemetry("LeftFollower", m_leftFollower);
        collectExtraMotorTelemetry("RightLeader", m_rightMotor);
        collectExtraMotorTelemetry("RightFollower", m_rightFollower);
    }

    private void collectExtraMotorTelemetry(String name, WPI_TalonFX motor)
    {
        var sensor = motor.getSensorCollection();

        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/Temperature", motor.getTemperature());
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/OutputPercent", motor.getMotorOutputPercent());
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/OutputVoltage", motor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/Sensor/PositionMeters", sensor.getIntegratedSensorPosition() * kSensorToMetersRatio);
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/Sensor/Position", sensor.getIntegratedSensorPosition());
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/Sensor/Velocity", sensor.getIntegratedSensorVelocity());
        SmartDashboard.putNumber("Drivebase/Motors/" + name + "/Sensor/PositionAbsolute", sensor.getIntegratedSensorAbsolutePosition());
    }

    public void reset() {
        resetMaxSpeed();
        for (var motor : m_motors) {
            motor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        }
    }

    public void arcadeDrive(double speed, double rotation) {
        m_robotDrive.arcadeDrive(speed, -rotation);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void setMaxSpeed(double maxOutput) {
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void resetMaxSpeed() {
        m_robotDrive.setMaxOutput(m_defaultSpeed);
    }

    public double getLeftDistanceMeters() {
        return m_leftMotor.getSelectedSensorPosition() * kSensorToMetersRatio;
    }

    public double getRightDistanceMeters() {
        return m_rightMotor.getSelectedSensorPosition() * kSensorToMetersRatio;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            m_leftMotor.getSelectedSensorVelocity() * kSensorToMetersRatio,
            m_rightMotor.getSelectedSensorVelocity() * kSensorToMetersRatio
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
    }
}
