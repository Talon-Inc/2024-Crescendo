// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private PneumaticHub pneumaticHub = new PneumaticHub();
  private DoubleSolenoid intake_Piston = pneumaticHub.makeDoubleSolenoid(OPEN_CHANNEL, CLOSE_CHANNEL);
  private DoubleSolenoid actuator_Piston = pneumaticHub.makeDoubleSolenoid(OPEN_CHANNEL2, CLOSE_CHANNEL2);
} 

public Pneumatics() {
  pneumaticHub.enableCompressorDigital();
}

public void Open(){
intake_Piston.set(DoubleSolenoid.Value.kForward);
}

public void Close(){
intake_Piston.set(DoubleSolenoid.Value.kReverse);
}

public void Up(){
 actuator_Piston.set(DoubleSolenoid.Value.kForward); 
}

public void Down(){
actuator_Piston.set(DoubleSolenoid.Value.kReverse); 
}
