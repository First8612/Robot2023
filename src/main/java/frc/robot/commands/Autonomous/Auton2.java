package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.Intake.IntakeSpeedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Auton2 extends SequentialCommandGroup {

    public Auton2(Drivetrain drivetrain, Intake intake, AHRS gyro) {
        addRequirements(drivetrain);
        addRequirements(intake);
        addCommands(
            //start up intake to eject game piece
            new IntakeSpeedCommand(-0.2, intake),
            //keep ghe intake going for 1 second
            new WaitCommand(1),
            //stop the intake
            new IntakeSpeedCommand(0, intake),
            //drive forward until past the charge station
            new DriveDistanceCommand(2.5, drivetrain, gyro),
            //turn -90 degrees
            new TurnDegreesCommand(0.5, 90, drivetrain, gyro),
            //drive to be parallel to the charging station
            new DriveDistanceCommand(2, drivetrain, gyro),
            //turn 90 degrees
            new TurnDegreesCommand(0.5, 90, drivetrain, gyro),
            //drive until tilted on the charging station
            new DriveUntilTiltCommand(15, 0.5, gyro, drivetrain),
            //balance on the charging station
            new BalanceCommand(gyro, drivetrain)
        );
    }
}
