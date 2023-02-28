package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeSpeedCommand extends CommandBase {
    private Intake intake;
    private double speed;

    public IntakeSpeedCommand(double speed, Intake intake) {
        super();
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }
}
