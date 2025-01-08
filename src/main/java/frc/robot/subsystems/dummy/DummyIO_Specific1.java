package frc.robot.subsystems.dummy;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DummyIO_Specific1 implements DummyIO {
  // Declare the subsystem specific hardware here
  // Example:
  private final SparkMax motor1;
  private final SparkMaxConfig config1;

  public DummyIO_Specific1() {
    // Instantiante and initilaze the substem according to Specific1 configuration.
    // This is specific to that robot (Motors, Sensors, controllers...)
    // Example:
    // Setup configuration for the motor
    config1 = new SparkMaxConfig();
    config1.inverted(false);

    // Create th motor and assign configuration
    motor1 = new SparkMax(DummyConstants.kvalue1, MotorType.kBrushless);
    motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
