package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    private double m_speed;

    public Intake() {
        super();

        m_intakeSolenoid.set(Value.kReverse);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void toggleIntake() {
        m_intakeSolenoid.toggle();
    }

    public void setSpeed(double speed) {
        m_speed = speed;
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(m_speed);
        SmartDashboard.putString("Intake Position", m_intakeSolenoid.get().name());
        SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
    }
}
