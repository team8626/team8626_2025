// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.presets.Presets;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.DoubleSupplier;

public class WristSetAngle extends CS_Command {
  private WristSubsystem wrist;
  private DoubleSupplier angle;
  private boolean overrideAngle = false;
  private double dashboardAngle = 0;
  private double desiredAngle = Presets.ALGAE_SHOOTBARGE_OURSIDE.getWristAngleDegrees();

  public WristSetAngle(DoubleSupplier newAngle) {
    wrist = RobotContainer.wrist;
    angle = newAngle;

    addRequirements(wrist);
    SmartDashboard.putBoolean("Commands/WristSetAngle/atSetPoint", false);

    this.setTAGString("WRIST_SETANGLE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = angle.getAsDouble();

    overrideAngle = SmartDashboard.getBoolean("Commands/AlgaeShooterRampUp/OverrideAngle", false);
    dashboardAngle = SmartDashboard.getNumber("Commands/AlgaeShooterRampUp/ForcedAngle", 0);
    if (overrideAngle) {
      desiredAngle = dashboardAngle;
    }
    wrist.setAngleDegrees(desiredAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetPoint = false;

    double currentAngle = wrist.getAngleDegrees();

    if (Math.abs(currentAngle - angle.getAsDouble()) <= WristConstants.toleranceDegrees) {
      atSetPoint = true;
    }
    SmartDashboard.putBoolean("Commands/WristSetAngle/atSetPoint", atSetPoint);
    return atSetPoint;
  }
}
