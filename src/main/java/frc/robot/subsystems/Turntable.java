package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turntable extends SubsystemBase {
    private CANSparkMax m_turntableMotor = new CANSparkMax(6, MotorType.kBrushless);
    private String buttonPressed;
    private double motorSpeed = 0.0;

    public void enableForwardTable() {
        buttonPressed = "forward";
        motorSpeed = 0.8;
    }  

    public void enableBackwardTable() {
        buttonPressed = "backward";
        motorSpeed = -0.8;
    }  

    public void disableTable() {
        buttonPressed = "stop";
        motorSpeed = 0.0;
    }  

    @Override
    public void periodic() {
        if (buttonPressed == "forward")
        m_turntableMotor.set(motorSpeed);
        else if (buttonPressed == "backward")
        m_turntableMotor.set(motorSpeed);
        else if (buttonPressed == "stop")
        m_turntableMotor.set(motorSpeed);
    }
    
}