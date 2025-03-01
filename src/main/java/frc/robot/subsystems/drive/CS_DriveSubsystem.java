package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.utils.CS_XboxController;
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

  public void setDefaultCommand(CS_XboxController xboxController) {
    subsystemInterface.setDefaultCommand(xboxController);
  }

  public void driveToPose() {
    subsystemInterface.driveToPose(null);
  }

  public Pose2d getPose2d() {
    return subsystemInterface.getPose();
  }

  public void drive(ChassisSpeeds velocity) {
    subsystemInterface.drive(velocity);
  }

  @Override
  public void CS_periodic() {}
}
