package frc.robot.driverProfiles;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

public class TankDriveProfile extends DriverProfileBase {
    private final Drivetrain drivetrain;
    private XboxController controller;
    private POVButton spinLeft;
    private JoystickButton spinRight;

    public TankDriveProfile(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    private void configureBindings() {
        controller = new XboxController(0);
        spinLeft = new POVButton(controller, 270);
        spinRight = new JoystickButton(controller, XboxController.Button.kB.value);

        spinLeft.whileTrue(new RunCommand(() -> {
            drivetrain.tankDrive(-1, 1);
        }, drivetrain));

        spinRight.whileTrue(new RunCommand(() -> {
            drivetrain.tankDrive(1, -1);
        }, drivetrain));
    }

    @Override
    public Command getTeleopCommand() {
        return new RunCommand(() -> {
            drivetrain.tankDrive(-controller.getRawAxis(XboxController.Axis.kLeftY.value), -controller.getRawAxis(5));
        }, drivetrain);
    }

    @Override
    public void enable() {
        this.configureBindings();       
    }

    @Override
    public void disable() {
        controller = null;
        spinRight = null;
        spinLeft = null;
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
