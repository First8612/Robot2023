package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public Intake() {
        super();

        m_intakeSolenoid.set(Value.kForward);
    }

    public void toggleIntake() {
        m_intakeSolenoid.toggle();
        new PrintCommand("Intake Toggled").schedule();
    }

    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Position", m_intakeSolenoid.get().name());
        SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
    }
}
