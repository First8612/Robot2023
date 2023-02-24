package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectoryCommand extends RamseteCommand {
    private Drivetrain drivetrain;
    private Trajectory trajectory;
    private Consumer<Pose2d> setPose;

    public FollowTrajectoryCommand(Supplier<Pose2d> getPose, Consumer<Pose2d> setPose, DifferentialDriveKinematics kinematics, Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            getPose,
            new RamseteController(2, .7),
            drivetrain.getFeedforward(),
            kinematics,
            drivetrain::getSpeeds,
            drivetrain.getLeftPIDController(),
            drivetrain.getRightPIDController(),
            drivetrain::setOutputVolts,
            drivetrain
        );
        this.setPose = setPose;
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        // set the robots pose to the initial post of the trajectory.
        // how do we do that (could pass in a Consumer<Pose2d> from robot container)
        // but how do we keep it from getting updated in RobotContainer.robotPeriodic?
        // might need to look at some examples.
        // the odometry.resetPosition might be helpful?
        setPose.accept(trajectory.getInitialPose());
        drivetrain.reset();
        // drivetrain.setDriveSafety(false); // https://www.chiefdelphi.com/t/bug-or-misunderstanding-in-trajectory-tutorial-and-ramsete-example/373989/5
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // drivetrain.setDriveSafety(true);
        drivetrain.setOutputVolts(0, 0);
    }
}
