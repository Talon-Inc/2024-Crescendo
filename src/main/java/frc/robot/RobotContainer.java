// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagAiming;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.GettingInRangeAT;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.OuttakeNote;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

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
  private final Intake m_intake = new Intake();
  private final LED m_led = new LED();

  // The robot's commands
  private final AprilTagAiming aprilTagAiming = new AprilTagAiming(m_robotDrive, m_Limelight);
  private final GettingInRangeAT gettingInRangeAT1 = new GettingInRangeAT(m_robotDrive, m_Limelight, 1.3, 2);
  private final GettingInRangeAT gettingInRangeAT2 = new GettingInRangeAT(m_robotDrive, m_Limelight, 3, 2);

  private final ClimbDownCommand climbDown = new ClimbDownCommand(m_Climb);
  private final ClimbUpCommand climbUp = new ClimbUpCommand(m_Climb);
  private final Shoot shoot = new Shoot(m_Shooter, m_intake);
  private final IntakeNote intakeNote = new IntakeNote(m_intake, m_led);
  private final OuttakeNote outtakeNote = new OuttakeNote(m_intake);
  private final AutoShoot autoShoot = new AutoShoot(m_Shooter, m_intake);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();

    //Register the Named Commands in Path Planner
    NamedCommands.registerCommand("shoot", autoShoot);
    NamedCommands.registerCommand("intakeNote", intakeNote);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser.addOption("P 1-Note; High", AutoBuilder.buildAuto("P 1-Note; High"));
    autoChooser.addOption("P 1-Note; Low", AutoBuilder.buildAuto("P 1-Note; Low  "));
    autoChooser.addOption("P 2-Note; High", AutoBuilder.buildAuto("P 2-Note; High"));
    autoChooser.addOption("P 2-Note; Low", AutoBuilder.buildAuto("P 2-Note; Low"));
    autoChooser.addOption("P 2-Note; Mid", AutoBuilder.buildAuto("P 2-Note; Mid"));
    autoChooser.addOption("Shoot Test", AutoBuilder.buildAuto("Shoot Test"));
    autoChooser.addOption("Fun and Games", AutoBuilder.buildAuto("Fun And Games"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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
    // A button (Resets the field relativity)
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // B button
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(aprilTagAiming);

    // X button
    new JoystickButton(m_driverController, Button.kX.value)
        .toggleOnTrue(intakeNote);

    // Y button
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(gettingInRangeAT1);

    // Start button
    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(climbUp);

    // Back button
    new JoystickButton(m_driverController, Button.kBack.value)
        .whileTrue(climbDown);

    // Left bumper
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new ParallelCommandGroup(shoot, new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive)));

    // Right bumper
    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whileTrue(outtakeNote);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
