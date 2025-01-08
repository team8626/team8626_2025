package frc.robot.subsystems.dummy;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DummyIO_Sim implements DummyIO {
  // Declare the subsystem specific hardware here
  // Example:
  private final SparkMax motor1;
  // private final AbsoluteEncoder encoder1;
  private final SparkClosedLoopController pidController1;
  // private final DCMotorSim motor1Sim;

  public DummyIO_Sim() {
    // Instantiante and initilaze the substem according to Specific1 configuration.
    // This is specific to that robot (Motors, Sensors, controllers...)
    // Example:

    // TODO: Migrate to 2025
    motor1 = new SparkMax(DummyConstants.kvalue1, MotorType.kBrushless);
    // encoder1 = motor1.getAbsoluteEncoder(Type.kDutyCycle);
    pidController1 = motor1.getClosedLoopController();
    // motor1Sim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0005);
  }

  @Override
  public void updateInputs(DummyIO_Inputs inputs) {
    // inputs.subsystemNameInput1 = 0.0;
    // inputs.subsystemNameInput2 = 0.0;
    // inputs.subsystemNameInput3 = true;
  }

  @Override
  public void setValue1(double value1) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setValue1'");
  }

  @Override
  public double getValue1() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getValue1'");
  }
}
