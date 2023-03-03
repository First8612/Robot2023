package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeSpeedCommand extends CommandBase {
    private Intake intake;
    private double speed;
    private Timer timer;
    private double seconds;

    public IntakeSpeedCommand(double speed, double seconds, Intake intake) {
        super();
        this.seconds = seconds;
        this.intake = intake;
        this.speed = speed;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }
}
