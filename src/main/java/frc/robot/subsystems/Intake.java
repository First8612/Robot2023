package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    private String intakeStatus;
    private double motorSpeed = 0;

    public void spinIn() {
        intakeStatus = "In";
        motorSpeed = 0.2;
    }

    public void spinOut() {
        intakeStatus = "Out";
        motorSpeed = -0.2;
    }

    public void stopSpin() {
        intakeStatus = "Stopped";
        motorSpeed = 0;
    }

    @Override
    public void periodic() {
        if (intakeStatus == "In") {
            m_intakeMotor.set(motorSpeed);
        } else if (intakeStatus == "Out") {
            m_intakeMotor.set(motorSpeed);
        } else if (intakeStatus == "Stopped") {
            m_intakeMotor.set(motorSpeed);
        }
    }
}
