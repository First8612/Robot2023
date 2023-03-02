package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives a set number of meters and stops. 
 * Uses PID controllers on the drivetrain so it SHOULD correct for straightness.
 */
public class DriveDistanceCommand extends CommandBase {
    private Drivetrain drivetrain;
    private double distanceMeters;
    private double startingRotation;

    private PIDController speedController;
    private PIDController rotationController;
    private Gyro gyro;

    public DriveDistanceCommand(double distanceMeters, Drivetrain drivetrain, Gyro gyro) {
        super();
        this.distanceMeters = distanceMeters;
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        speedController = new PIDController(3, 0, 0);
        rotationController = new PIDController(0, 0, 0);
    }

    private double getAverageDistance() {
        return (this.drivetrain.getLeftDistanceMeters() + this.drivetrain.getRightDistanceMeters()) / 2;
    }

    @Override
    public void initialize() {
        super.initialize();

        startingRotation = gyro.getAngle();
        speedController.setSetpoint(distanceMeters);
    }

    @Override
    public void execute() {
        super.execute();
    
        var speed = speedController.calculate(getAverageDistance());
        speed = Math.min(speed, 0.5);
        var rotationError = startingRotation - gyro.getAngle();
        var rotation = rotationController.calculate(rotationError);

        drivetrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished() {
        double distanceToGo = distanceMeters - getAverageDistance();
        return distanceToGo < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        drivetrain.setOutputVolts(0, 0);
    }
}
