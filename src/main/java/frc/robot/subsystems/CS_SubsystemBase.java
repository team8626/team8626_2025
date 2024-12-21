package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.UpdateInterval;

/**
 * The CSSubsystemBase class is an abstract base class for subsystems 
 * created by Team 8626 (Cyber Sailors).
 * 
 * <p> It extends the SubsystemBase class and provides additional functionality 
 * for registering subsystems with the Dashboard and setting dashboars and simulation
 * methods in a more intuitive manner.
 *
 * <p> This class is designed to streamline the creation and management of subsystems within the robot code.
*/
public abstract class CS_SubsystemBase extends SubsystemBase{

    /** MasterSubsystem Constructor
     * This registers the subsystem with the Dashboard.
     * If the subsystem implements its own constructor, it should call this constructor.
     * or take care of the registration itself.
     *
     * <p>This can be done using super(SubsystemName);
     *
     * @param interval The update interval for the subsystem (SHORT_INTERVAL or LONG_INTERVAL)
     *                 By default (no parameter) the update interval is "SHORT_INTERVAL"
     */
    public CS_SubsystemBase(UpdateInterval interval) {
        Dashboard.registerSubsystem(this);
        this.initDashboard();
    }
    public CS_SubsystemBase() {
        this(UpdateInterval.SHORT_INTERVAL);
    }

    /**
     * Print a formatted string prefixed with the subsystem name.
     *
     * @param format The format string.
     * @param args   The arguments referenced by the format specifiers in the format string.
     */
    protected void printf(String format, Object... args) {
        String className = this.getClass().getSimpleName().toUpperCase();
        String formattedMessage = String.format(format, args);
        System.out.println("[" + className + "] " + formattedMessage);
    }

    /**
     * Print a given string prefixed with the subsystem name.
     *
     * @param string The string to print.
     */
    protected void println(String string) {
        this.printf(string);
    }

    /**
     * Abstract method to initialize the Dashboard.
     * Subclasses must implement this method.
     */
    public abstract void initDashboard();

    /**
     * Abstract method to update the Dashboard.
     * Subclasses must implement this method.
     */
    public abstract void updateDashboard();

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     * <p> Team 8626 has overridden this method to call the {@link #CS_periodic()} method 
     *  and update simulation values.
     * 
     */
    @Override
    public final void periodic() {
        // if(Robot.isSimulation()) {
        //     this.updateSimValues();
        // }
        this.CS_periodic();
    }

    /**
     * Abstract method to update the subsystem periodically.
     * This replaces the {@link #periodic()} method in the SubsystemBase class.
     * and is called periodically after updating simulation values if needed
     * <p> Subclasses must implement this method.
     */
    public abstract void CS_periodic();
}
