// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.presets.Presets;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.Supplier;

public class WristSetAngle extends CS_Command {
  private WristSubsystem wrist;
  private Supplier<Angle> angle;
  private boolean overrideAngle = false;
  private Angle dashboardAngle = Degrees.of(0);
  private Angle desiredAngle = Presets.ALGAE_SHOOTBARGE_OURSIDE.getWristAngle();

  public WristSetAngle(Supplier<Angle> newAngle) {
    wrist = RobotContainer.wrist;
    angle = newAngle;

    addRequirements(wrist);
    SmartDashboard.putBoolean("Commands/WristSetAngle/atSetPoint", false);

    this.setTAGString("WRIST_SETANGLE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = angle.get();

    overrideAngle = SmartDashboard.getBoolean("Commands/AlgaeShooterRampUp/OverrideAngle", false);
    dashboardAngle =
        Degrees.of(SmartDashboard.getNumber("Commands/AlgaeShooterRampUp/ForcedAngle", 0));
    if (overrideAngle) {
      desiredAngle = dashboardAngle;
    }
    wrist.setAngle(desiredAngle);
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

    Angle currentAngle = wrist.getAngle();

    if (currentAngle.isNear(desiredAngle, WristConstants.tolerance)) {
      atSetPoint = true;
    }
    SmartDashboard.putBoolean("Commands/WristSetAngle/atSetPoint", atSetPoint);
    return atSetPoint;
  }
}
