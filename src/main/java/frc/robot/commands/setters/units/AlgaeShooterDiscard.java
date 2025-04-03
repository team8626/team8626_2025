// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;

public class AlgaeShooterDiscard extends CS_Command {
  private AlgaeShooterSubsystem algae501;
  private final Timer timer = new Timer();

  public AlgaeShooterDiscard() {
    algae501 = RobotContainer.algae501;

    this.setTAGString("ALGAESHOOTER_DISCARD");
  }

  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.ALGAE_DISCARD);

    algae501.startLauncher(AlgaeShooterConstants.launcherShootSetpoint);
    algae501.startShooterBySetpoint(AlgaeShooterConstants.discardShootSetpoint);
    timer.reset();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    algae501.stopLauncher();
    algae501.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
