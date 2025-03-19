package frc.robot;

import java.util.Arrays;
import java.util.List;

public class UIConstants {
  public static final List<CORAL_BRANCH> ALLOWED_CORAL_BRANCHES =
      Arrays.asList(
          CORAL_BRANCH.A,
          CORAL_BRANCH.B,
          CORAL_BRANCH.C,
          CORAL_BRANCH.D,
          CORAL_BRANCH.E,
          CORAL_BRANCH.F,
          CORAL_BRANCH.G,
          CORAL_BRANCH.H,
          CORAL_BRANCH.I,
          CORAL_BRANCH.J,
          CORAL_BRANCH.K,
          CORAL_BRANCH.L);

  public static final List<ALGAE_FACE> ALLOWED_ALGAE_FACES =
      Arrays.asList(
          ALGAE_FACE.AB, ALGAE_FACE.CD, ALGAE_FACE.EF, ALGAE_FACE.GH, ALGAE_FACE.IJ, ALGAE_FACE.KL);

  public static final List<CORAL_LEVEL> ALLOWED_CORAL_LEVELS =
      Arrays.asList(CORAL_LEVEL.L1, CORAL_LEVEL.L3, CORAL_LEVEL.L4);

  public static final DTP defaultDTP = DTP.OFF;
  public static final DTP defaultDealgaefyDTP = DTP.OFF;
  public static final DTP defaultAlgaeShootDTP = DTP.OFF;
  public static final CORAL_LEVEL defaultCoralLevel = CORAL_LEVEL.L4;
  public static final PICKUP_SIDE defaultPickupSide = PICKUP_SIDE.NONE;
  public static final CORAL_BRANCH defaultCoralBranch = CORAL_BRANCH.NONE;
  public static final ALGAE_FACE defaultAlgaeFace = ALGAE_FACE.FLOOR;

  public enum ALLIANCE_COLOR {
    UNKNOWN(0),
    RED(1),
    BLUE(2);
    private final int value;

    private ALLIANCE_COLOR(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public enum PICKUP_SIDE {
    NONE(0),
    LEFT(1),
    RIGHT(2);
    private final int value;

    private PICKUP_SIDE(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }

    public static PICKUP_SIDE getByValue(int value) {
      for (PICKUP_SIDE val : PICKUP_SIDE.values()) {
        if (val.getValue() == value) {
          return val;
        }
      }
      throw new IllegalArgumentException("No enum constant with value " + value);
    }
  }

  public enum DTP {
    OFF(false),
    ON(true);
    private final boolean value;

    private DTP(boolean value) {
      this.value = value;
    }

    public boolean getValue() {
      return value;
    }

    public static DTP getByValue(boolean value) {
      for (DTP val : DTP.values()) {
        if (val.getValue() == value) {
          return val;
        }
      }
      throw new IllegalArgumentException("No enum constant with value " + value);
    }
  }

  public enum CORAL_LEVEL {
    L1(1),
    L2(2),
    L3(3),
    L4(4);
    private final int value;

    private CORAL_LEVEL(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }

    public static CORAL_LEVEL getByValue(int value) {
      for (CORAL_LEVEL level : CORAL_LEVEL.values()) {
        if (level.getValue() == value) {
          return level;
        }
      }
      throw new IllegalArgumentException("No enum constant with value " + value);
    }
  }

  public enum CORAL_BRANCH {
    NONE(0),
    A(1),
    B(2),
    C(3),
    D(4),
    E(5),
    F(6),
    G(7),
    H(8),
    I(9),
    J(10),
    K(11),
    L(12);
    private final int value;

    private CORAL_BRANCH(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }

    public static CORAL_BRANCH getByValue(int value) {
      for (CORAL_BRANCH branch : CORAL_BRANCH.values()) {
        if (branch.getValue() == value) {
          return branch;
        }
      }
      throw new IllegalArgumentException("No enum constant with value " + value);
    }
  }

  public enum ALGAE_FACE {
    FLOOR(0),
    AB(1),
    CD(2),
    EF(3),
    GH(4),
    IJ(5),
    KL(6);

    private final int value;

    private ALGAE_FACE(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }

    public static ALGAE_FACE getByValue(int value) {
      for (ALGAE_FACE face : ALGAE_FACE.values()) {
        if (face.getValue() == value) {
          return face;
        }
      }
      throw new IllegalArgumentException("No enum constant with value " + value);
    }
  }
}
