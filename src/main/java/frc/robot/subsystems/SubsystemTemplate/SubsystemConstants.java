package frc.robot.subsystems.SubsystemTemplate;

public class SubsystemConstants {
  public class SubsystemStates {
    public static enum SubsystemState {
      IDLE(0, "idle"),
      STATE_1(1, "State1"),
      STATE_2(2, "State2"),
      STATE_3(3, "State3");

      private final int m_id;
      private final String m_string;

      SubsystemState(int newId, String newString) {
        this.m_id = newId;
        this.m_string = newString.toUpperCase();
      }

      public String getString() {
        return m_string.toUpperCase();
      }

      public double getId() {
        return m_id;
      }
    }
  }
}
