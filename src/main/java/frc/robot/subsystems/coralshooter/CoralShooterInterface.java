package frc.robot.subsystems.coralshooter;

public interface CoralShooterInterface {
    void startShooter(double new_RPM);
    void updateShooterRPM(double new_RPM);
    void stopShooter();

    void startLauncher(double new_RPM);
    void updateLauncherRPM(double new_RPM);
    void stopLauncher();

    boolean shooterIsLoaded();

    double getShooterRPMLeft();
    double getShooterRPMRight();
    double getLauncherRPM();

    /** Run flywheels at voltage */
    default void runCharacterizationLeft(double input) {}
    default void runCharacterizationRight(double input) {}
    
    default void setPID(double kP, double kI, double kD) {}

    default void updateInputs(CoralShooterValues values) {
        // Default implementation
    }

    public class CoralShooterValues {
        protected boolean launchIsEnabled = false;  
        protected boolean shooterIsEnabled = false;

        protected double currentRPMLeft = 0; // RPM
        protected double currentRPMRight = 0; // RPM
        protected double currentRMPLauncher = 0; // RPM

        protected double ampsLeft = 0;
        protected double ampsRight = 0;
        protected double ampsLauncher = 0; 

        protected boolean isLoaded = false;
        protected double desiredRPM = CoralShooterConstants.shooterRPM;

        protected double kP = 0.05;
        protected double kI = 0.0;
        protected double kD = 0.0;
        protected double FF = 0.0;
    }
}