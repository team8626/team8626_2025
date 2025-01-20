package frc.robot.subsystems.coralshooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;
import frc.utils.CS_Utils;

public class CoralShooterSubsystem extends CS_SubsystemBase {
    private CoralShooterInterface coralShooterInterface;
    private CoralShooterValues values;

    public CoralShooterSubsystem(CoralShooterInterface subsystem_interface) {
    super();

    this.coralShooterInterface = subsystem_interface;
    values = new CoralShooterValues();
    println("Created");
    }

    // Calls to the coralShooter interface
    public void stop() {
    coralShooterInterface.stopShooter();
    }

    public void setSpeed(double new_height) {
    coralShooterInterface.setShooterRPM(new_height);
    }

    public double getShooterRPMLeft() {
        return coralShooterInterface.getShooterRPMLeft();
    }
    public double getShooterRPMRight() {
        return coralShooterInterface.getShooterRPMRight();
    }

    public void setPID(double newkP, double newkI, double newkD) {
        coralShooterInterface.setPID(newkP, newkI, newkD);
    }

    public void setkP(double newkP) {
        coralShooterInterface.setPID(newkP, values.kI, values.kD);
    }
    public void setkI(double newkI) {
        coralShooterInterface.setPID(values.kP, newkI, values.kD);
    }
    public void setkD(double newkD) {
        coralShooterInterface.setPID(values.kP, values.kI, newkD);
    }

    @Override
    public void CS_periodic() {
        coralShooterInterface.updateInputs(values);
    }
    
    @Override
    public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/CoralShooter/P Gain", CoralShooterConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/CoralShooter/I Gain", CoralShooterConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/CoralShooter/D Gain", CoralShooterConstants.gains.kD());
    }

    @Override
    public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/CoralShooter/Enabled", values.is_enabled);
    SmartDashboard.putNumber("Subsystem/CoralShooter/Left RPM", values.current_speed_left);
    SmartDashboard.putNumber("Subsystem/CoralShooter/Right RPM", values.current_speed_right);

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/CoralShooter/P Gain", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/CoralShooter/I Gain", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/CoralShooter/D Gain", values.kD);
    // double newFF = SmartDashboard.getNumber("Subsystem/CoralShooter/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // System.out.printf("P: %f, I: %f, D: %f, FF: %f\n", values.kP, values.kI, values.kD, values.FF); 

    }

    @Override
    public void simulationPeriodic() {
    // TODO Auto-generated method stub

    }
    
}
