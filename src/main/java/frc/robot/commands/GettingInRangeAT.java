// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class GettingInRangeAT extends Command {
  double Kp = 0.5000;
  // double Kp_rot = -.0076;
  double maxSpeed = .4;

  double currentDistanceZ;
  double desiredDistance;
  int tagID;

  DriveSubsystem swerveDrive;
  Limelight limelight;

  /** Creates a new GettingInRangeAT. */
  public GettingInRangeAT(DriveSubsystem swerveDrive, Limelight limelight, double desiredDistance, int tagID) {

    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    this.desiredDistance = desiredDistance;
    this.tagID = tagID;

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
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

      double[] botpose_targetspace = limelight.getBotPositionToTargetSpace();
      double currentDistanceZ = -botpose_targetspace[2];
      double currentDistanceX = -botpose_targetspace[0];
      double distance = Math.hypot(currentDistanceX, currentDistanceZ);

      double distanceError = desiredDistance - distance;

      double speedX = distanceError * Kp;
      // double speedY = distanceErrorX * Kp;
      // double rot = tx * Kp_rot;

      if (tid != tagID) {
        tv = 0;
      }

      if (tv == 0) {
        speedX = 0;
        // speedY = 0;
        // rot = 0;
      }
      
      // I don't want the robot to go a very fast speedY/X if it is very far away from robot
      if (Math.abs(speedX) > maxSpeed) {
        speedX = maxSpeed;
      }

      // if (Math.abs(speedY) > maxSpeed) {
      //   speedY = maxSpeed;
      // }

      // if (Math.abs(distanceErrorX) < 0.1) {
      //   speedY = 0;
      // }

      if (Math.abs(distanceError) < 0.1) {
        speedX = 0;
      }
    
    swerveDrive.drive(speedX, 0, 0, false, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.turnOffLED();
    swerveDrive.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
