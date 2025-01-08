package frc.robot.subsystems.dummy;

public interface DummyIO {
  public static class DummyIO_Inputs {
    public double subsystemNameInput1 = 0;
    public double subsystemNameInput2 = 0;
    public boolean subsystemNameInput3 = false;
  }

  public default void updateInputs(DummyIO_Inputs inputs) {}

  public abstract void setValue1(double value1);

  public abstract double getValue1();
}
