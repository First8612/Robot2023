package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private String intakeStatus;
    private double motorSpeed = 0;
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public void spinIn() {
        intakeStatus = "In";
        motorSpeed = 0.5;
    }

    public void spinOut() {
        intakeStatus = "Out";
        motorSpeed = -0.5;
    }

    public void slowIn() {
        intakeStatus = "Slow In";
        motorSpeed = 0.2;
    }

    public void slowOut() {
        intakeStatus = "Slow Out";
        motorSpeed = -0.2;
    }

    public void toggleIntake() {
        m_intakeSolenoid.toggle();
    }

    @Override
    public void periodic() {
        if (intakeStatus == "In") {
            m_intakeMotor.set(motorSpeed);
        } else if (intakeStatus == "Out") {
            m_intakeMotor.set(motorSpeed);
        } else if (intakeStatus == "Slow In") {
            m_intakeMotor.set(motorSpeed);
        } else if (intakeStatus == "Slow Out") {
            m_intakeMotor.set(motorSpeed);
        }
    }
}
