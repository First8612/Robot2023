package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.Intake.IntakeSpeedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Auton3 extends SequentialCommandGroup {
    public Auton3(Drivetrain drivetrain, Intake intake, AHRS gyro) {
        addRequirements(drivetrain);
        addRequirements(intake);
        addCommands(
            //start up intake to eject game piece
            new IntakeSpeedCommand(0.5, 1, intake),
            //wait 1 second
            new WaitCommand(1),
            //drive past the charging station
            new DriveDistanceCommand(-1.75, drivetrain, gyro),
            //balance on the charging station
            new BalanceCommand(gyro, drivetrain)
        );
    }
}
