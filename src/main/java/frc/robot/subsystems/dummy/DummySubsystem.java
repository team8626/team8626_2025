// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dummy;

import frc.robot.subsystems.CS_SubsystemBase;
// import frc.robot.subsystems.Dummy.DummyIO.DummyIO_Inputs;

public class DummySubsystem extends CS_SubsystemBase {
  DummyIO m_interface;
  // DummyIO_Inputs m_inputs = new DummyIO_Inputs();

  public DummySubsystem(DummyIO subsystem_interface) {
    this.m_interface = subsystem_interface;
    println("Created");
  }

  public void setValue1(double new_value1) {
    m_interface.setValue1(new_value1);
  }

  public double getValue1() {
    return m_interface.getValue1();
  }

  @Override
  public void initDashboard() {
    // TODO Auto-generated method stub
  }

  @Override
  public void updateDashboard() {
    // SmartDashboard.putData("Commands/DummySubsystem", this);
  }

  @Override
  public void CS_periodic() {}
}
