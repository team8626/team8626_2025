// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * The CS_Command class is an abstract base class for commands created by Team 8626 (Cyber Sailors).
 *
 * <p>It extends the Command class and provides additional functionality such as printing to the
 * console using prefix.
 *
 * <p>This class is designed to streamline the creation and management of commands within the robot
 * code.
 */
public abstract class CS_Command extends Command {
  private String TAG = "";

  public CS_Command() {
    super();
    TAG = this.getClass().getSimpleName().toUpperCase();
  }

  /**
   * Print a formatted string prefixed with the subsystem name.
   *
   * @param format The format string.
   * @param args The arguments referenced by the format specifiers in the format string.
   */
  protected void printf(String format, Object... args) {
    String formattedMessage = String.format(format, args);
    if (RobotContainer.isDebugEnabled()) {
      System.out.println("[" + TAG + "] " + formattedMessage);
    }
  }

  /**
   * Print a given string prefixed with the subsystem name.
   *
   * @param string The string to print.
   */
  protected void println(String string) {
    this.printf(string);
  }

  protected void setTAGString(String tag) {
    TAG = tag;
  }
}
