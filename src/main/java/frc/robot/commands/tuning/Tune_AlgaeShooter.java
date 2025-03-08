// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.Presets;

public class Tune_AlgaeShooter extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  public Tune_AlgaeShooter() {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);

    this.setTAGString("TUNE_ALGAESHOOTER");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algae501.startRampUp(Presets.ALGAE_NETFROM10FT.getRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      algae501.stopShooter();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
