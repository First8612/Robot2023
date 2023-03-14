package frc.robot.driverProfiles;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

public class NateAndDrewProfile extends DriverProfileBase {
    private final Drivetrain drivetrain;
    private XboxController controller;

    public NateAndDrewProfile(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    private void configureBindings() {
        controller = new XboxController(0); 
    }

    @Override
    public Command getTeleopCommand() {
        return new RunCommand(() -> {
            var forward = controller.getRawAxis(3);
            var reverse = -controller.getRawAxis(2);
            var speed = forward + reverse;
            drivetrain.arcadeDrive(
                speed, 
                controller.getRawAxis(XboxController.Axis.kLeftX.value) * 0.75
            );
        }, drivetrain);
    }

    @Override
    public void enable() {
        this.configureBindings();       
    }

    @Override
    public void disable() {
        controller = null;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
