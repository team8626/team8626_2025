// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorConstants;
import frc.utils.CS_XboxController;

public class CS_DriveSubsystemIO_Tank extends TankSubsystem implements CS_DriveSubsystemIO {

  public CS_DriveSubsystemIO_Tank() {}

  /**
   * Sets the drive command for the swerve subsystem using the provided Xbox controller. The command
   * uses the left joystick for forward/backward and strafe movements, and the right joystick for
   * rotation.
   *
   * @param xboxController The Xbox controller to use for driving the robot.
   */
  public void setDefaultCommand(CS_XboxController xboxController) {
    Command driveCommand =
        run(
            () ->
                this.tankDrive(
                    MathUtil.applyDeadband(
                        -xboxController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                    MathUtil.applyDeadband(
                        -xboxController.getRightY(), OperatorConstants.LEFT_X_DEADBAND)));

    setDefaultCommand(driveCommand);
  }

  @Override
  public Pose3d getPose3d() {
    // TODO Auto-generated method stub
    return new Pose3d();
  }
}
