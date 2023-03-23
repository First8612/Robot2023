package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Auton4 extends SequentialCommandGroup{
    public Auton4(Drivetrain drivetrain, Intake intake, AHRS gyro, Shooter shooter) {
        addRequirements(drivetrain);
        addRequirements(intake);
        addRequirements(shooter);
        addCommands(
            new RunCommand(() -> { shooter.shooterEject(-0.8); }).withTimeout(1),
            new InstantCommand(() -> { shooter.shooterStop(); }),
            //drive out past the charging station
            new DriveDistanceCommand(4, drivetrain, gyro)
        );
    }  
}
