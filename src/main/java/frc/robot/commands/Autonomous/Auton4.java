package frc.robot.commands.Autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Auton4 extends SequentialCommandGroup{
    public Auton4(Drivetrain drivetrain, Intake intake, AHRS gyro, Shooter shooter, Conveyor conveyor) {
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
                        intake.setSpeed(0.5); // is this the right direction, and will it stay running?
                        conveyor.setSpeed(1); // is this the right direction, speed, and will it stay running?
                    })
                ),   

                //drive out past the charging station
                new DriveDistanceCommand(-5, drivetrain, gyro)
            ),

            // retract and stop the intake
            new InstantCommand(() -> {
                intake.toggleIntake();
                intake.setSpeed(0); // stop the intake
            }),

            // drive back a little less then we drove out
            new DriveDistanceCommand(4.5, drivetrain, gyro), 

            // yeet
            new RunCommand(() -> { 
                shooter.shooterEject(0.8); 
            }).withTimeout(5)
        );
    }  
}
