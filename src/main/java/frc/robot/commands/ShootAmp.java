// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final LED led;
  private final Timer timer;

  /** Creates a new ShootAmp. */
  public ShootAmp(Shooter shooter, Intake intake, LED led) {
    this.led = led;
    this.shooter = shooter;
    this.intake = intake;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shootAmp();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterAtAmpSpeed() || timer.hasElapsed(2)) {
      Timer.delay(0.1);
      intake.moveIntakeChannel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
    led.setBlack();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
