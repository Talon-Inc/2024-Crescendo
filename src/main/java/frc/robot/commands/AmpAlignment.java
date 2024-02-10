// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AprilTagAiming;
import frc.robot.commands.GettingInRangeAT;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AmpAlignment extends Command {

  DriveSubsystem swerveDrive;
  Limelight limelight;
  double currentDistanceZ;
  double desiredZ;
  double desiredX;

  /** Creates a new AmpAlignment. */
  public AmpAlignment(DriveSubsystem swerveDrive, Limelight limelight, double desiredZ, double desiredX) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    this.desiredZ = desiredZ;
    this.desiredX = desiredX;


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
