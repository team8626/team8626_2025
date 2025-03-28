// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.units.measure.Distance;
import java.util.function.Consumer;

/**
 * Team 8626 (Cyber Sailors) utility class.
 *
 * <p>This is a collections of utility methods that are used across the robot code.
 */
public class CS_Utils {

  /**
   * Helper method to update from SmartDashboard values
   *
   * @param newValue The new value to be updated
   * @param oldValue The old value to be updated
   * @param updateFunction The function to be called to update the value
   * @return The new value if it has been updated, otherwise the old value
   */
  public static double updateFromSmartDashboard(
      double newValue, double oldValue, Consumer<Double> updateFunction) {
    double retVal = oldValue;
    if (newValue != oldValue) {
      updateFunction.accept(newValue);
      retVal = newValue;
    }
    return retVal;
  }

  public static int updateFromSmartDashboard(
      int newValue, int oldValue, Consumer<Integer> updateFunction) {
    int retVal = oldValue;
    if (newValue != oldValue) {
      updateFunction.accept(newValue);
      retVal = newValue;
    }
    return retVal;
  }

  public static Distance clamp(Distance value, Distance low, Distance high) {
    return max(low, max(value, high));
  }

  public static Distance max(Distance a, Distance b) {
    return a.gt(b) ? a : b;
  }

  public static Distance min(Distance a, Distance b) {
    return a.lt(b) ? a : b;
  }
}
