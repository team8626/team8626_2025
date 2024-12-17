package frc.robot.subsystems.Dummy;

import frc.robot.subsystems.CS_SubsystemBase;
// import frc.robot.subsystems.Dummy.DummyIO.DummyIO_Inputs;

public class DummySubsystem extends CS_SubsystemBase {
    DummyIO m_interface;
    // DummyIO_Inputs m_inputs = new DummyIO_Inputs();

    public DummySubsystem(DummyIO subsystem_interface){
        this.m_interface = subsystem_interface;
    }

    public void setValue1(double new_value1){
        m_interface.setValue1(new_value1);
    }

    public double getValue1(){
        return m_interface.getValue1();
    }

    @Override
    public void initDashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initDashboard'");
    }

    @Override
    public void updateDashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateDashboard'");
    }

    @Override
    public void updateSimValues() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateSimValues'");
    }

    @Override
    public void CS_periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'CS_periodic'");
    }
}
