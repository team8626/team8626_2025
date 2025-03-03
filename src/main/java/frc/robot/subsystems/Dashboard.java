// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;

/**
 * The Dashboard class manages the registration and periodic updating of subsystems with different
 * update intervals. It extends the SubsystemBase class.
 *
 * <p>Subsystems can be registered with either a short update interval (20ms) or a long update
 * interval (500ms). The class maintains separate lists for subsystems with different update
 * intervals and updates them accordingly.
 *
 * @see CS_SubsystemBase
 * @see edu.wpi.first.wpilibj2.command.SubsystemBase
 */
public class Dashboard extends CS_SubsystemBase {

  private static List<CS_SubsystemBase> subsystemsShortInterval = new ArrayList<>();
  private static List<CS_SubsystemBase> subsystemsLongInterval = new ArrayList<>();

  private static final boolean kEnableDashBoard = true;

  private static final double kShortInterval = .02; //  20ms
  private static final double kLongInterval = .5; // 500ms

  private double m_shortOldTime = 0.0;
  private double m_longOldTime = 0.0;

  public enum GamePieceState {
    RAMPING_UP,
    LAUNCHING,
    INTAKING,
    IDLE,
    LOADED
  }

  private static GamePieceState coralState = GamePieceState.IDLE;
  private static GamePieceState algaeState = GamePieceState.IDLE;

  /** Use assignment types for updateDashboard implementation. */
  public enum UpdateInterval {
    SHORT_INTERVAL,
    LONG_INTERVAL
  }

  public Dashboard() {}
  /**
   * Registers a subsystem with the specified update interval.
   *
   * @param subsystem The subsystem to register.
   * @param interval The update interval for the subsystem (SHORT_INTERVAL or LONG_INTERVAL). By
   *     default (no parameter) the update interval is SHORT_INTERVAL
   */
  public static void registerSubsystem(CS_SubsystemBase subsystem, UpdateInterval interval) {
    if (interval == UpdateInterval.LONG_INTERVAL) {
      subsystemsLongInterval.add(subsystem);
    } else {
      subsystemsShortInterval.add(subsystem);
    }
    System.out.println(
        "[DASHBOARD] Registered "
            + subsystem.getClass().getName()
            + " for "
            + interval
            + " updates");
  }

  /**
   * Registers a subsystem with the default update interval (SHORT_INTERVAL).
   *
   * @param subsystem The subsystem to register.
   */
  public static void registerSubsystem(CS_SubsystemBase subsystem) {
    registerSubsystem(subsystem, UpdateInterval.SHORT_INTERVAL);
  }

  public static List<CS_SubsystemBase> listRegisteredSubsystems() {
    List<CS_SubsystemBase> combinedIntervals = new ArrayList<>();
    combinedIntervals.addAll(subsystemsShortInterval);
    combinedIntervals.addAll(subsystemsLongInterval);
    return combinedIntervals;
  }

  @Override
  public void CS_periodic() {
    double time = Timer.getFPGATimestamp();
    if (kEnableDashBoard) {
      // Update short interval
      if ((time - m_shortOldTime) > kShortInterval) {
        m_shortOldTime = time;
        for (CS_SubsystemBase s : subsystemsShortInterval) {
          s.updateDashboard();
        }
      }
      // Update long interval
      if ((time - m_longOldTime) > kLongInterval) {
        // Thing that should be updated every LONG_DELAY
        m_longOldTime = time;
        for (CS_SubsystemBase s : subsystemsShortInterval) {
          s.updateDashboard();
        }
        for (CS_SubsystemBase s : subsystemsLongInterval) {
          s.updateDashboard();
        }
      }
    }

    // Get Subsystem Steady States
    // (Could be a preloaded piece or a reboot...)
    if (coralState == GamePieceState.IDLE || coralState == GamePieceState.LOADED) {
      if (RobotContainer.mortar != null && RobotContainer.mortar.isLoaded()) {
        coralState = GamePieceState.LOADED;
      } else {
        coralState = GamePieceState.IDLE;
      }
    }
    if (algaeState == GamePieceState.IDLE || algaeState == GamePieceState.LOADED) {
      if (RobotContainer.algae501 != null && RobotContainer.algae501.isLoaded()) {
        algaeState = GamePieceState.LOADED;
      } else {
        algaeState = GamePieceState.IDLE;
      }
    }
  }

  public static void setCoralState(GamePieceState new_state) {
    coralState = new_state;
  }

  public static GamePieceState getCoralState() {
    return coralState;
  }

  public static GamePieceState getAlgaeState() {
    return algaeState;
  }

  public static void setAlgaeState(GamePieceState new_state) {
    algaeState = new_state;
  }

  public static Command getSetCoralStateCommand(GamePieceState new_state) {
    return new InstantCommand(() -> Dashboard.setCoralState(new_state));
  }

  public static Command getSetAlgaeStateCommand(GamePieceState new_state) {
    return new InstantCommand(() -> Dashboard.setAlgaeState(new_state));
  }
}
