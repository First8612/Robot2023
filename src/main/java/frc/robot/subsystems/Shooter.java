package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMotor = new CANSparkMax(7, MotorType.kBrushless);

    public void shooterEject(double speed) {
        m_shooterMotor.set(speed);
    } 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", m_shooterMotor.get());
    }
    
}
