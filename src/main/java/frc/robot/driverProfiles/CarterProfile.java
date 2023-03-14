package frc.robot.driverProfiles;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;

public class CarterProfile extends DriverProfileBase {
    private Drivetrain drivetrain;
    private XboxController controller;
    private JoystickButton spinRight;
    private JoystickButton spinLeft;

    public CarterProfile(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    private void configureBindings() {
        controller = new XboxController(0);
        spinRight = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        spinLeft = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        
        spinRight.whileTrue(new RunCommand(() -> {
            drivetrain.arcadeDrive(0, -1);
        }));
        spinLeft.whileTrue(new RunCommand(() -> {
            drivetrain.arcadeDrive(0, 1);
        }));
    }

    @Override
    public Command getTeleopCommand() {
        return new RunCommand(() -> {
            drivetrain.arcadeDrive(-controller.getRawAxis(Axis.kLeftY.value), controller.getRawAxis(Axis.kRightX.value));
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
