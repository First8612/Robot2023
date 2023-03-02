// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesCommand extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final AHRS m_gyro;
  private final double m_degrees;
  private final double m_speed;
  private Supplier<Boolean> m_isRedAlliance;

  private final PIDController turnController;

  public TurnDegreesCommand(double speed, double degrees, Drivetrain drivetrain, AHRS gyro, Supplier<Boolean> isRedAlliance) {
    m_degrees = degrees;
    m_speed = speed;
    m_drivetrain = drivetrain;
    m_gyro = gyro;
    m_isRedAlliance = isRedAlliance;

    addRequirements(drivetrain);

    turnController = new PIDController(0.015, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drivetrain.arcadeDrive(0, 0);
    m_drivetrain.reset();
    turnController.setSetpoint(m_degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Math.min(m_speed, turnController.calculate(m_gyro.getYaw()));
    m_drivetrain.arcadeDrive(0, speed);
    var allianceAwareDegrees = (m_isRedAlliance.get() ? 1 : -1) * m_degrees;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_degrees - m_gyro.getYaw()) < 10;
  }

}
