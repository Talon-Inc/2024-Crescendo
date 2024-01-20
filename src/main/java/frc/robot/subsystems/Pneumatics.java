// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

public class Pneumatics extends SubsystemBase {
  private PneumaticHub p_pneumaticHub = new PneumaticHub();
  private DoubleSolenoid p_actuatorLeft = p_pneumaticHub.makeDoubleSolenoid(kActuatorLeftOpen, kActuatorLeftClose);
  private DoubleSolenoid p_actuatorRight = p_pneumaticHub.makeDoubleSolenoid(kActuatorRightClose, kActuatorRightOpen);
  
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    p_pneumaticHub.enableCompressorDigital();
  }

  public void up() {
    p_actuatorLeft.set(DoubleSolenoid.Value.kForward);
    p_actuatorRight.set(DoubleSolenoid.Value.kForward); 
  }

  public void down() {
    p_actuatorLeft.set(DoubleSolenoid.Value.kReverse);
    p_actuatorRight.set(DoubleSolenoid.Value.kReverse); 
  }
}
