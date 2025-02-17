// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.CS_XboxController;

public interface CS_DriveSubsystemIO {
  public void setDefaultCommand(CS_XboxController xboxController);

  public Pose3d getPose3d();
}
