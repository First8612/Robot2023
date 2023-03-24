package frc.robot.commands.Autonomous;

import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.subsystems.*;

public class Auton4 extends SequentialCommandGroup{
    public Auton4(Drivetrain drivetrain, Intake intake, AHRS gyro, Shooter shooter, Conveyor conveyor, Supplier<Boolean> isRedAlliance) {
        addRequirements(drivetrain);
        addRequirements(intake);
        addRequirements(shooter);
        addRequirements(conveyor);
        addCommands(
            // yeet
            new RunCommand(() -> { shooter.shooterEject(0.8); }).withTimeout(1),
            // stop shooter
            new InstantCommand(() -> { shooter.shooterStop(); }),
            
            new ParallelCommandGroup(          
                // wait, then extend the intake, start the intake, and start feeding the conveyor
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new InstantCommand(() -> { 
                        intake.toggleIntake();
                        intake.setSpeed(-0.5);
                        conveyor.setSpeed(0.5);
                    }),
                    new WaitCommand(3)
                ),   

                //drive out past the charging station
                new DriveDistanceCommand(-4.75, drivetrain, gyro)
            ),

            // retract and stop the intake
            new InstantCommand(() -> {
                intake.toggleIntake();
                intake.setSpeed(0);
            }),

            // drive back a little less then we drove out
            new DriveDistanceCommand(4, drivetrain, gyro),

            //turn
            //new TurnDegreesCommand(0.3, 20, drivetrain, gyro, isRedAlliance),

            //stop the conveyor
            new InstantCommand(() -> {
                conveyor.setSpeed(0);
            }),

            //shoot cube out
            new RunCommand(() -> {
                shooter.shooterEject(0.8);
            }).withTimeout(1));
    }  
}
