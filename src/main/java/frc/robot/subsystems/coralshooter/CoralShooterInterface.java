package frc.robot.subsystems.coralshooter;

public interface CoralShooterInterface {
    void setShooterRPM(double new_speed);
    void stopShooter();

    void setLauncherSpeed(double new_speed);
    void stopLauncher();

    double getShooterRPMLeft();
    double getShooterRPMRight();

    default void setPID(double kP, double kI, double kD) {}

    default void updateInputs(CoralShooterValues values) {
        // Default implementation
    }

    public class CoralShooterValues {
        protected boolean launchIsEnabled = false;  
        protected boolean shooterIsEnabled = false;

        protected double currentRPMLeft = 0; // RPM
        protected double currentRPMRight = 0; // RPM
        protected double currentLauncherSpeed = 0; // [-1 ; 1]]

        protected double kP = 0.05;
        protected double kI = 0.0;
        protected double kD = 0.0;
        protected double FF = 0.0;
    }
}