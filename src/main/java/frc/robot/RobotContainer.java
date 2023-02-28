// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.Intake.IntakeSpeedCommand;
import frc.robot.subsystems.*;
import java.util.function.Consumer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final double kChassisWidthMeters = Units.inchesToMeters(23);
  private final AHRS m_gyro = new AHRS(Port.kMXP);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kChassisWidthMeters);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
  private Pose2d pose = new Pose2d();

  private final int m_axisForwardBack = XboxController.Axis.kLeftY.value;
  private final int m_axisLeftRight = XboxController.Axis.kRightX.value;
  private final int m_intakeAxis = XboxController.Axis.kLeftY.value;
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private final POVButton m_intakeOutCone = new POVButton(m_operatorController, 0);
  private final POVButton m_intakeInCone = new POVButton(m_operatorController, 180);
  private final JoystickButton m_intakeOutCube = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton m_intakeInCube = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton m_intakeToggle = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);

  private final POVButton m_turntableForwardButton = new POVButton(m_operatorController, 90);
  private final POVButton m_turntableBackwardButton = new POVButton(m_operatorController, 270);

  private final JoystickButton m_balanceButton = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);

  private Field2d field = new Field2d();
  private CommandBase resetPosition;
  
  // The robot's subsystems and commands are defined here...  
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Turntable m_turntable = new Turntable();
  private final BalanceCommand m_balance = new BalanceCommand(m_gyro, m_robotDrive);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    loadTrajectories(m_chooser);
    SmartDashboard.putData("Auton Chooser", m_chooser);
    SmartDashboard.putData(field);

    this.resetPosition = new InstantCommand(() -> {
      m_gyro.reset();
      m_robotDrive.reset();
      odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
    }).withName("Reset Position");

    SmartDashboard.putData(resetPosition);
    resetPosition.schedule();
  }

  public void robotPeriodic() {
    pose = odometry.update(m_gyro.getRotation2d(), m_robotDrive.getLeftDistanceMeters(), m_robotDrive.getRightDistanceMeters());
    field.setRobotPose(pose);
  }

  public void teleopInit() {
    resetPosition.schedule();

    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          double speed = m_driverController.getRawAxis(m_axisLeftRight);
          double rotation = m_driverController.getRawAxis(m_axisForwardBack);
          m_robotDrive.arcadeDrive(-speed, rotation); 
        },
        m_robotDrive));

    m_intake.setDefaultCommand(
      new RunCommand(() -> {
        m_intake.setSpeed(m_operatorController.getRawAxis(m_intakeAxis));
      },
      m_intake
    ));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_intakeInCone.whileTrue(new IntakeSpeedCommand(0.5, m_intake));

    m_intakeOutCone.whileTrue(new IntakeSpeedCommand(-0.5, m_intake));

    m_intakeInCube.whileTrue(new IntakeSpeedCommand(0.2, m_intake));

    m_intakeOutCube.whileTrue(new IntakeSpeedCommand(-0.2, m_intake));

    m_turntableForwardButton.whileTrue(new RunCommand(() -> {
      m_turntable.enableForwardTable();
    }));
  
    m_turntableBackwardButton.whileTrue(new RunCommand(() -> {
      m_turntable.enableBackwardTable();
    }));

    m_intakeToggle.onTrue(new InstantCommand(() -> {
      m_intake.toggleIntake();
    }));

    m_balanceButton.onTrue(m_balance);
  }

  private void loadTrajectories(SendableChooser<Command> chooser) {
    Consumer<Pose2d> setPose = pose -> odometry.resetPosition(new Rotation2d(), 0, 0, pose);

    var path1 = PathPlanner.loadPath("TestPath1", new PathConstraints(.5, .3));
    var trajectory1Command = new FollowTrajectoryCommand(() -> pose, setPose, kinematics, m_robotDrive, path1);
    chooser.setDefaultOption("Path 1", trajectory1Command);

    var path2 = PathPlanner.loadPath("TestPath2", new PathConstraints(.5, .3));
    var trajectory2Command = new FollowTrajectoryCommand(() -> pose, setPose, kinematics, m_robotDrive, path2);
    chooser.addOption("Path 2", trajectory2Command);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command will be run in autonomous
    m_robotDrive.reset();
    return m_chooser.getSelected();
  }
}
