package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives a set number of meters and stops. 
 * Uses PID controllers on the drivetrain so it SHOULD correct for straightness.
 * TODO: Does line 36/39 correctly output in the right scale so line 41 can use it right?
 */
public class DriveDistanceCommand extends CommandBase {
    private Drivetrain drivetrain;
    private double distanceMeters;
    private double startingDistanceLeft;
    private double startingDistanceRight;

    public DriveDistanceCommand(double distanceMeters, Drivetrain drivetrain) {
        super();
        this.distanceMeters = distanceMeters;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        super.initialize();

        startingDistanceLeft = this.drivetrain.getLeftDistanceMeters();
        startingDistanceRight = this.drivetrain.getRightDistanceMeters();
    }

    @Override
    public void execute() {
        super.execute();

        var leftDistanceToGo = (distanceMeters - startingDistanceLeft) - (drivetrain.getLeftDistanceMeters() - startingDistanceLeft);
        var leftSpeed = drivetrain.getLeftPIDController().calculate(leftDistanceToGo);
       
        var rightDistanceToGo = (distanceMeters - startingDistanceRight) - (drivetrain.getRightDistanceMeters() - startingDistanceRight);
        var rightSpeed = drivetrain.getRightPIDController().calculate(rightDistanceToGo);
 
        drivetrain.setOutputVolts(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        drivetrain.setOutputVolts(0, 0);
    }
}
