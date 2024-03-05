// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {

  Intake intake;
  LED led;
  /** Creates a new Intake. */
  public IntakeNote(Intake intake) {
    this.intake = intake;
    // this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getNoteVoltage() > 1) {
      intake.stop();
      // led.setGreen();
    } else {
      intake.intakeNote();
      // led.setBlack();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
