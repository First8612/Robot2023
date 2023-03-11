package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SpeedLimit;
import frc.robot.subsystems.Drivetrain;

public class HoldPositionCommand extends CommandBase {
    private SpeedLimit speedLimit = new SpeedLimit(0.5);
    private PIDController leftController = new PIDController(3, 0, 0);
    private PIDController rightController = new PIDController(3, 0, 0);
    private Drivetrain drivetrain;

    public HoldPositionCommand(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        leftController.setSetpoint(drivetrain.getLeftDistanceMeters());
        rightController.setSetpoint(drivetrain.getRightDistanceMeters());
    }

    @Override
    public void execute() {
        var leftPosition = drivetrain.getLeftDistanceMeters();
        var rightPosition = drivetrain.getRightDistanceMeters();

        var leftSpeed = speedLimit.apply(leftController.calculate(leftPosition));
        var rightSpeed = speedLimit.apply(rightController.calculate(rightPosition));

        drivetrain.tankDrive(leftSpeed, rightSpeed, false);
    }

}
