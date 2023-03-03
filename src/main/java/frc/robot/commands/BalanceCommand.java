package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import com.kauailabs.navx.frc.AHRS;

public class BalanceCommand extends PIDCommand {
    private static PIDController controller = new PIDController(
        0.04, 
        0, 
        0
    );

    public BalanceCommand(AHRS gyro, Drivetrain drivetrain) {
        super(
            controller, 
            gyro::getPitch, // --> measures the angle
            0, 
            value -> {
                drivetrain.arcadeDrive(
                    Math.min(-value, 0.32), // drive speed with ceiling
                    0 // drive rotation
                );
            },
            drivetrain
        );
        SmartDashboard.putData(controller);
    }

}
