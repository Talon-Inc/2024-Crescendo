// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class OuttakeNote extends Command {
  private final Intake intake;
  private final LED led;

  /** Creates a new OuttakeNote. */
  public OuttakeNote(Intake intake, LED led) {
    this.led = led;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.outtakeNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setBlack();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
