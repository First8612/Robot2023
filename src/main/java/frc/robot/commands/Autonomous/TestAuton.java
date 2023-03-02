package frc.robot.commands.Autonomous;

import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TestAuton extends SequentialCommandGroup {
    public TestAuton(Drivetrain drivetrain, Intake intake, AHRS gyro, Supplier<Boolean> isRedAlliance) {
        addRequirements(drivetrain);
        addRequirements(intake);
        addCommands(
            new DriveDistanceCommand(2, drivetrain, gyro),

            new WaitCommand(2),

            new TurnDegreesCommand(0.5, 90, drivetrain, gyro, isRedAlliance)
        );
    }
}
