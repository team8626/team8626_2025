// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.Presets;

public class Tune_CoralShooter extends CS_Command {
  private CoralShooterSubsystem mortar;

  public Tune_CoralShooter() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("TUNE_CORALSHOOTER");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mortar.startRampUp(Presets.CORAL_L4.getRPMLeft(), Presets.CORAL_L4.getRPMRight());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      mortar.stopShooter();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
