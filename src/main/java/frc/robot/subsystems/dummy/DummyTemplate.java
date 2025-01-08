package frc.robot.subsystems.dummy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.dummy.DummyConstants.SubsystemStates.SubsystemState;

public class DummyTemplate extends CS_SubsystemBase {

  private boolean m_enabled;
  private SubsystemState m_currentState;

  public DummyTemplate() {
    super();
    this.println("Has been created");
  }

  @Override
  public void initDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    // SmartDashboard.getNumber("Subsystem/P Gain", m_P);
    // SmartDashboard.getNumber("Subsystem/D Gain", m_D);
    // SmartDashboard.getNumber("Subsystem/I Gain", m_I);
    // SmartDashboard.getNumber("Subsystem/FF Gain", m_FF);

    // TODO Auto-generated method stub

  }

  @Override
  public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/ENABLED", m_enabled);
    SmartDashboard.putString("Subsystem/State", m_currentState.getString());

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    // double newP = SmartDashboard.getNumber("Subsystem/P Gain", defaultP);
    // double newI = SmartDashboard.getNumber("Subsystem/I Gain", defaultI);
    // double newD = SmartDashboard.getNumber("Subsystem/D Gain", defaultD);
    // double newFF = SmartDashboard.getNumber("Subsystem/D Gain", defaultFF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    // m_P = Utils8626.updateFromSmartDashboard(newP, m_P, (value) ->
    // somePIDController.setP(value));
    // m_I = Utils8626.updateFromSmartDashboard(newI, m_I, (value) ->
    // somePIDController.setP(value));
    // m_D = Utils8626.updateFromSmartDashboard(newD, m_D, (value) ->
    // somePIDController.setP(value));
    // m_FF = Utils8626.updateFromSmartDashboard(newFF, m_FF, (value) ->
    // somePIDController.setP(value));

    // TODO Auto-generated method stub

  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public void CS_periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'CS_periodic'");
  }
}
