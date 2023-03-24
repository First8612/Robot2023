package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private CANSparkMax m_conveyorMotor = new CANSparkMax(6, MotorType.kBrushless);

    public Conveyor() {
        super();

        m_conveyorMotor.restoreFactoryDefaults();
    }

    public void setSpeed(double speed) {
        m_conveyorMotor.set(speed);
    }  

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Conveyor Speed", m_conveyorMotor.get());
    }
    
}