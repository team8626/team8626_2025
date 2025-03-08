// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorConstants;
import frc.utils.CS_XboxController;
import java.io.File;
import java.util.function.Supplier;

public class CS_DriveSubsystemIO_Swerve extends SwerveSubsystem implements CS_DriveSubsystemIO {

  public CS_DriveSubsystemIO_Swerve(File directory) {
    super(directory);
  }
  /**
   * Sets the drive command for the swerve subsystem using the provided Xbox controller. The command
   * uses the left joystick for forward/backward and strafe movements, and the right joystick for
   * rotation.
   *
   * @param xboxController The Xbox controller to use for driving the robot.
   */
  public void setDefaultCommand(CS_XboxController xboxController) {
    Command driveCommand =
        this.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -xboxController.getLeftY(), // * (AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                    OperatorConstants.LEFT_Y_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -xboxController.getLeftX(), // * (AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> -xboxController.getRightX());

    setDefaultCommand(driveCommand);
  }

  @Override
  public Command driveToPose(Supplier<Pose2d> newPoseSupplier) {
    printf("Driving to Pose: %s", newPoseSupplier.get());
    return super.driveToPose(newPoseSupplier.get());
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose2(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            4.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
  }
}
