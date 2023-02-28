package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveUntilTiltCommand extends CommandBase{
    private double goalTiltAngle;
    private Drivetrain drivetrain;

    private ProfiledPIDController driveController;
    private AHRS gyro;

    public DriveUntilTiltCommand(double goalTiltAngle, double driveSpeed, AHRS gyro, Drivetrain drivetrain) {
        super();
        this.goalTiltAngle = goalTiltAngle;
        this.gyro = gyro;
        this.drivetrain = drivetrain;

        driveController = new ProfiledPIDController(0.6, 0, 0, new TrapezoidProfile.Constraints(driveSpeed, 0.2));
    }

    @Override
    public void execute() {
        // not going to drive straight.
        var speed = driveController.calculate(this.gyro.getRawGyroX(), goalTiltAngle);

        drivetrain.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setOutputVolts(0, 0);
    }
}

