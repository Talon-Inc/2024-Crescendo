// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends CANSparkMax {

  SimDouble m_CANSparkMaxSimAppliedOutput;

  SimDeviceSim m_CANSparkMaxSim;

  /** Creates a new SimulatableCANSparkMax. */
  public SimulatableCANSparkMax(int deviceID, MotorType type) {
    super(deviceID, type);

    m_CANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceID);
    m_CANSparkMaxSimAppliedOutput = m_CANSparkMaxSim.getDouble("Applied Output");


  }

  @Override
  public void set(double speed) {
    super.set(speed);
  }
}
