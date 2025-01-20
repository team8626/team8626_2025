package frc.robot.subsystems.coralshooter;

public interface CoralShooterInterface {
    void stopShooter();
    void setShooterRPM(double new_speed);
    double getShooterRPMLeft();
    double getShooterRPMRight();

    default void setPID(double kP, double kI, double kD) {}

    default void updateInputs(CoralShooterValues values) {
        // Default implementation
    }

    public class CoralShooterValues {
        protected boolean is_enabled = false;
        protected double current_speed_left = 0; // RPM
        protected double current_speed_right = 0; // RPM

        protected double kP = 0.05;
        protected double kI = 0.0;
        protected double kD = 0.0;
        protected double FF = 0.0;
    }
}