package frc.robot.subsystems.dummy;

public class DummyConstants {
  public class SubsystemStates {
    public static enum SubsystemState {
      IDLE(0, "idle"),
      STATE_1(1, "State1"),
      STATE_2(2, "State2"),
      STATE_3(3, "State3");

      private final int id;
      private final String string;

      SubsystemState(int newId, String newString) {
        this.id = newId;
        this.string = newString.toUpperCase();
      }

      public String getString() {
        return string.toUpperCase();
      }

      public double getId() {
        return id;
      }
    }
  }

  public static int kvalue1 = 0;
}
