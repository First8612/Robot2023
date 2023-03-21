package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.Intake.IntakeSpeedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Auton5 extends SequentialCommandGroup {
    public Auton5(Drivetrain drivetrain, Intake intake, AHRS gyro, Shooter shooter) {
        addRequirements(drivetrain, intake, shooter);
        addCommands(
            new RunCommand(() -> { shooter.shooterEject(-0.8); }).withTimeout(5),
            new InstantCommand(() -> { shooter.shooterStop(); }),
            //start up intake to eject game piece
            new IntakeSpeedCommand(0.5, 1, intake),
            //wait 1 second
            new WaitCommand(1),
            //drive past the charging station
            new DriveDistanceCommand(-4.5, drivetrain, gyro),
            //drive back to the charging station
            new DriveDistanceCommand(1.75, drivetrain, gyro),
            //balance on the charging station
            new BalanceCommand(gyro, drivetrain)
        );
    }
}

