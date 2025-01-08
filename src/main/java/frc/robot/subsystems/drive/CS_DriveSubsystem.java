package frc.robot.subsystems.drive;

import frc.robot.subsystems.CS_SubsystemBase;
import java.io.File;

// public class CS_SwerveSubsystem extends SwerveSubsystem  implements CS_SwerveSubsystemIO{
public class CS_DriveSubsystem extends CS_SubsystemBase {
  private CS_DriveSubsystemIO subsystemInterface;

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface) {
    this.subsystemInterface = subsystem_interface;
    // TODO: Instantiate the Drivetrain
  }

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface, File directory) {
    this.subsystemInterface = subsystem_interface;
    // TODO: Instantiate the Drivetrain
    // super(directory);
  }

  @Override
  public void initDashboard() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'initDashboard'");
  }

  @Override
  public void updateDashboard() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateDashboard'");
  }

  @Override
  public void CS_periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'CS_periodic'");
  }
}
