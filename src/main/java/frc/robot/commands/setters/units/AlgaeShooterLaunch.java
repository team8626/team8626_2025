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
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;

public class AlgaeShooterLaunch extends CS_Command {
  private AlgaeShooterSubsystem algae501;
  private final Timer timer = new Timer();

  public AlgaeShooterLaunch() {
    algae501 = RobotContainer.algae501;

    // Do not use algae501 in requirements, this would cancel the Ramp-up Command!!!
    // addRequirements(algae501);

    this.setTAGString("ALGAESHOOTER_LAUNCH");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Dashboard.setAlgaeState(GamePieceState.LAUNCHING);
    Commodore.setCommodoreState(CommodoreState.ALGAE_SHOOT_LAUNCH);

    algae501.startLauncher(AlgaeShooterConstants.launcherShootSetpoint);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!algae501.isLoaded()) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dashboard.setAlgaeState(GamePieceState.IDLE);

    algae501.stopLauncher();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(AlgaeShooterConstants.launchTimerSeconds);
  }
}
