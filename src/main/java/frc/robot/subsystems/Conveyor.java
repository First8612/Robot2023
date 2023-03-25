package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private CANSparkMax m_conveyorMotor = new CANSparkMax(6, MotorType.kBrushless);
    private double m_speed;

    public Conveyor() {
        super();

        m_conveyorMotor.restoreFactoryDefaults();
    }

    public void setSpeed(double speed) {
        m_speed = speed;
    }  

    @Override
    public void periodic() {
        m_conveyorMotor.set(m_speed);
        SmartDashboard.putNumber("Conveyor Speed", m_conveyorMotor.get());
    }
    
}