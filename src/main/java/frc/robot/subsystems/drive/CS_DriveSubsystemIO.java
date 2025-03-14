// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.CS_XboxController;
import java.util.function.Supplier;

public interface CS_DriveSubsystemIO {

  public static class DriveValues {
    protected boolean isFipped = false;
    protected Pose2d robotPose = new Pose2d();
    protected Rotation2d robotHeading = new Rotation2d();
    protected Rotation2d robotPitch = new Rotation2d();
    protected ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    protected ChassisSpeeds robotVelocity = new ChassisSpeeds();
  }

  public void setDefaultCommand(CS_XboxController xboxController);

  // public Command driveToPose(Supplier<Pose2d> poseSupplier);

  public ChassisSpeeds getRobotVelocity();

  public Pose2d getPose();

  public void drive(ChassisSpeeds velocity);

  public void toggleFlip();

  public Command sysIdDriveMotorCommand();

  public Command sysIdAngleMotorCommand();

  public Command centerModulesCommand();

  public Command driveToPose(Supplier<Pose2d> pose);

  public default void updateInputs(DriveValues values) {}
}
