// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagAiming;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.GettingInRangeAT;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
// import frc.robot.commands.AlignAtAprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Limelight m_Limelight = new Limelight();
  private final Climb m_Climb = new Climb();
  private final Shooter m_Shooter = new Shooter();
  // private final Shooter m_Shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final LED m_led = new LED();

  // private final AlignAtAprilTag alignAtAprilTag = new AlignAtAprilTag(m_robotDrive, m_Limelight, 1, 1);
  private final AprilTagAiming aprilTagAiming = new AprilTagAiming(m_robotDrive, m_Limelight);
  private final GettingInRangeAT gettingInRangeAT1 = new GettingInRangeAT(m_robotDrive, m_Limelight, 2, 1, 5);
  private final GettingInRangeAT gettingInRangeAT2 = new GettingInRangeAT(m_robotDrive, m_Limelight, 3, 0, 5);
  //:3
  private final ClimbDownCommand climbDown = new ClimbDownCommand(m_Climb);
  private final ClimbUpCommand climbUp = new ClimbUpCommand(m_Climb);
  private final Shoot shoot = new Shoot(m_Shooter, m_intake);
  // private final Shoot shoot = new Shoot(m_Shooter, m_intake);
  private final IntakeNote intakeNote = new IntakeNote(m_intake);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    //Register the Named Commands in Path Planner
    NamedCommands.registerCommand("shoot", shoot);
    NamedCommands.registerCommand("intakeNote", intakeNote);

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser.addOption("StupidCrap", AutoBuilder.buildAuto("New Auto"));
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
            
    // Constantly run intake, even during autonomous
    // m_intake.setDefaultCommand(intakeNote);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    // The A button on controller (Resets the field relativity)
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // new JoystickButton(m_driverController, Button.kB.value)
     //   .whileTrue(alignAtAprilTag);

    // The B button on the controller
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(aprilTagAiming);

    // X button
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(climbDown);

    // Y button
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(climbUp);

    // Left bumper
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(shoot);

    // Start button
    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(intakeNote);


    //Right Bumper Button
    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    //     .whileTrue(shoot);

    // // The X button on controller
    // new JoystickButton(m_driverController, Button.kCircle.value)
    // .whileTrue(gettingInRangeAT2);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // m_robotDrive::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false, false));
  }
}
