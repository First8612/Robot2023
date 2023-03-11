package frc.robot.driverProfiles;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class DriverProfileBase {
    public abstract Command getTeleopCommand();
    public abstract void enable(); 
    public abstract void disable();
}