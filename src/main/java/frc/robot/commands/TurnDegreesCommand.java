// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
//-import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesCommand extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final AHRS m_gyro;
  private final double m_degrees;
  private final double m_speed;

  private final PIDController turnController;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesCommand(double speed, double degrees, Drivetrain drivetrain, AHRS gyro) {
    m_degrees = degrees;
    m_speed = speed;
    m_drivetrain = drivetrain;
    m_gyro = gyro;
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

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drivetrain.getLeftDistanceMeters());
    double rightDistance = Math.abs(m_drivetrain.getRightDistanceMeters());
    return (leftDistance + rightDistance) / 2.0;
  }
}
