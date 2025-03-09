package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.utils.CS_XboxController;
import java.io.File;

public class CS_DriveSubsystem extends CS_SubsystemBase {
  private CS_DriveSubsystemIO subsystemInterface;
  private boolean isFlipped = false;

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface) {
    this.subsystemInterface = subsystem_interface;
  }

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface, File directory) {
    this.subsystemInterface = subsystem_interface;
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

  public ChassisSpeeds getRobotVelocity() {
    return subsystemInterface.getRobotVelocity();
  }

  public void drive(ChassisSpeeds velocity) {
    subsystemInterface.drive(velocity);
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  public void flipToggle() {
    subsystemInterface.toggleFlip();
  }

  @Override
  public void CS_periodic() {}
}
