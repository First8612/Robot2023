package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveUntilTiltCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.commands.Intake.IntakeSpeedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Auton1 extends SequentialCommandGroup {

    public Auton1(Drivetrain drivetrain, Intake intake) {
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
            new DriveDistanceCommand(4, drivetrain, null), //add gyro
            //turn -90 degrees
            new TurnDegreesCommand(0.5, -90, drivetrain),
            //drive to be parallel to the charging station
            new DriveDistanceCommand(2, drivetrain, null),
            //turn 90 degrees
            new TurnDegreesCommand(0.5, -90, drivetrain),
            //drive until tilted on the charging station
            new DriveUntilTiltCommand(0, 0.5, null, drivetrain) //add gyro

        );
    }
}
