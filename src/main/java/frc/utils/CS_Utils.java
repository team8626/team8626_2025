package frc.utils;

import java.util.function.Consumer;

/**
 * Team 8626 (Cyber Sailors) utility class.
 * <p>This is a collections of utility methods that are used across the robot code.
 */
public class CS_Utils {

    /**
     * Helper method to update from SmartDashboard  values
     * 
     * @param newValue The new value to be updated
     * @param oldValue The old value to be updated
     * @param updateFunction The function to be called to update the value
     * 
     * @return The new value if it has been updated, otherwise the old value
     * 
     */
    public static double updateFromSmartDashboard(double newValue, double oldValue, Consumer<Double> updateFunction) {
        double retVal = oldValue;
        if (newValue != oldValue) {
            updateFunction.accept(newValue);
            retVal = newValue;
        }
        return retVal;
    }
}
