// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

/**
 * The Dashboard class manages the registration and periodic updating of subsystems
 * with different update intervals. It extends the SubsystemBase class.
 * 
 * <p>Subsystems can be registered with either a short update interval (20ms) or a long
 * update interval (500ms). The class maintains separate lists for subsystems with
 * different update intervals and updates them accordingly.
 * 
 * @see CSSubsystemBase
 * @see edu.wpi.first.wpilibj2.command.SubsystemBase
 */
public class Dashboard extends SubsystemBase {

  private static List<CSSubsystemBase> subsystemsShortInterval = new ArrayList<>();
  private static List<CSSubsystemBase> subsystemsLongInterval = new ArrayList<>();

  private static final boolean kEnableDashBoard = true;

  private static final double kShortInterval = .02; //  20ms
  private static final double kLongInterval  = .5; // 500ms

  private double m_shortOldTime = 0.0;
  private double m_longOldTime = 0.0;

  /**
   * Use assignment types for updateDashboard implementation.
   */
    public enum UpdateInterval {
    SHORT_INTERVAL,
    LONG_INTERVAL
  }

  /**
   * Registers a subsystem with the specified update interval.
   * 
   * @param subsystem The subsystem to register.
   * @param interval The update interval for the subsystem (SHORT_INTERVAL or LONG_INTERVAL).
  *                  By default (no parameter) the update interval is SHORT_INTERVAL
   */
  public static void registerSubsystem(CSSubsystemBase subsystem, UpdateInterval interval) {
      if(interval == UpdateInterval.LONG_INTERVAL){
          subsystemsLongInterval.add(subsystem);
      } else {
          subsystemsShortInterval.add(subsystem);
      }
      System.out.println("[DASHBOARD] Registered " + subsystem.getClass().getName() + " for " + interval + " updates");
  }

  /**
   * Registers a subsystem with the default update interval (SHORT_INTERVAL).
   * 
   * @param subsystem The subsystem to register.
   */
  public static void registerSubsystem(CSSubsystemBase subsystem) {
    registerSubsystem(subsystem, UpdateInterval.SHORT_INTERVAL);
  }

  public static List<CSSubsystemBase> listRegisteredSubsystems() {
    List<CSSubsystemBase> combinedIntervals = new ArrayList<>();
    combinedIntervals.addAll(subsystemsShortInterval);
    combinedIntervals.addAll(subsystemsLongInterval);
    return combinedIntervals;
  }

  @Override
  public void periodic() {
    double time = Timer.getFPGATimestamp();
    if (kEnableDashBoard) {
      // Update short interval
      if ((time - m_shortOldTime) > kShortInterval) {
        m_shortOldTime = time;
        for (CSSubsystemBase s: subsystemsShortInterval){
          s.updateDashboard();
        }
      }
      // Update long interval
      if ((time - m_longOldTime) > kLongInterval) {
        // Thing that should be updated every LONG_DELAY
        m_longOldTime = time;
        for (CSSubsystemBase s : subsystemsShortInterval) {
          s.updateDashboard();
        }
        for (CSSubsystemBase s : subsystemsLongInterval) {
          s.updateDashboard();
        }
      }
    }
  }
}
