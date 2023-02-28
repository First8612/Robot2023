package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public void toggleIntake() {
        m_intakeSolenoid.toggle();
    }

    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    @Override
    public void periodic() {

    }
}
