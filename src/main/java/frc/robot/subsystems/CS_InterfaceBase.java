// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * The CS_InterfaceBase interface is an interface base for interfaces created by Team 8626 (Cyber
 * Sailors).
 */
public interface CS_InterfaceBase {
  /**
   * Print a formatted string prefixed with the subsystem name.
   *
   * @param format The format string.
   * @param args The arguments referenced by the format specifiers in the format string.
   */
  default void printf(String format, Object... args) {
    String className = this.getClass().getSimpleName().toUpperCase();
    String formattedMessage = String.format(format, args);
    if (!Robot.isReady() || RobotContainer.isDebugEnabled()) {
      System.out.println("[" + className + "] " + formattedMessage);
    }
  }
  /**
   * Print a given string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  default void println(String string) {
    this.printf(string);
  }

  /**
   * Print a given DriverStation Warning string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  default void DSWarningf(String format, Object... args) {
    String className = this.getClass().getSimpleName().toUpperCase();
    String formattedMessage = String.format(format, args);
    DriverStation.reportWarning("[" + className + "] " + formattedMessage, false);
  }

  /**
   * Print a given DriverStation Warning string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  default void DSWarningln(String string) {
    DSWarningf(string);
  }
  /**
   * Print a given DriverStation Warning string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  default void DSErrorf(String format, Object... args) {
    String className = this.getClass().getSimpleName().toUpperCase();
    String formattedMessage = String.format(format, args);
    DriverStation.reportError("[" + className + "] " + formattedMessage, false);
  }

  /**
   * Print a given DriverStation Warning string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  default void DSErrorln(String string) {
    DSErrorf(string);
  }
}
