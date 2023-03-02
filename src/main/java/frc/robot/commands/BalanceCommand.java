package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import com.kauailabs.navx.frc.AHRS;

public class BalanceCommand extends PIDCommand {
    private static PIDController controller = new PIDController(
        0.05, 
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
                    -value, // drive speed
                    0 // drive rotation
                );
                System.out.println(gyro.getPitch());
            },
            drivetrain
        );
        SmartDashboard.putData(controller);
    }

}
