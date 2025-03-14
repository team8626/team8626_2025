package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO.DriveValues;
import frc.utils.CS_XboxController;
import java.io.File;
import java.util.function.Supplier;

public class CS_DriveSubsystem extends CS_SubsystemBase {
  private CS_DriveSubsystemIO driveInterface;
  private boolean isFlipped = false;
  private DriveValues values;

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface) {
    this.driveInterface = subsystem_interface;
    values = new DriveValues();
  }

  public CS_DriveSubsystem(CS_DriveSubsystemIO subsystem_interface, File directory) {
    this.driveInterface = subsystem_interface;
    values = new DriveValues();
  }

  public void setDefaultCommand(CS_XboxController xboxController) {
    driveInterface.setDefaultCommand(xboxController);
  }

  public Pose2d getPose2d() {
    return driveInterface.getPose();
  }

  public ChassisSpeeds getRobotVelocity() {
    return driveInterface.getRobotVelocity();
  }

  public void drive(ChassisSpeeds velocity) {
    driveInterface.drive(velocity);
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  public void flipToggle() {
    driveInterface.toggleFlip();
  }

  public Command sysIdAngleMotorCommand() {
    return driveInterface.sysIdAngleMotorCommand();
  }

  public Command sysIdDriveMotorCommand() {
    return driveInterface.sysIdDriveMotorCommand();
  }

  public Command centerModulesCommand() {
    return driveInterface.centerModulesCommand();
  }

  public Command driveToPose(Supplier<Pose2d> newPoseSupplier) {
    if ((newPoseSupplier == null) || newPoseSupplier.get() == null) {
      return new InstantCommand();
    } else {
      return driveInterface.driveToPose(newPoseSupplier);
    }
  }

  @Override
  public void CS_periodic() {
    driveInterface.updateInputs(values);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Subsystem/Drive/Flipped", values.isFipped);

    SmartDashboard.putNumber("Subsystem/Drive/Heading(deg)", values.robotHeading.getDegrees());
    SmartDashboard.putNumber("Subsystem/Drive/Pitch(deg)", values.robotPitch.getDegrees());

    SmartDashboard.putNumber(
        "Subsystem/Drive/FieldVelocity (m.s-1)",
        Math.sqrt(
            Math.pow(values.fieldVelocity.vxMetersPerSecond, 2.0)
                + Math.pow(values.fieldVelocity.vyMetersPerSecond, 2.0)));
    SmartDashboard.putNumber(
        "Subsystem/Drive/FieldVelocity (deg.s-1)",
        Units.radiansToDegrees(values.fieldVelocity.omegaRadiansPerSecond));
    SmartDashboard.putNumber(
        "Subsystem/Drive/RobotVelocity (m.s-1)",
        Math.sqrt(
            Math.pow(values.robotVelocity.vxMetersPerSecond, 2.0)
                + Math.pow(values.robotVelocity.vyMetersPerSecond, 2.0)));
    SmartDashboard.putNumber(
        "Subsystem/Drive/RobotVelocity (deg.s-1)",
        Units.radiansToDegrees(values.robotVelocity.omegaRadiansPerSecond));
  }
}
