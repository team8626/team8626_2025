// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterLaunch extends CS_Command {
  private CoralShooterSubsystem mortar;
  private final Timer timer = new Timer();

  public CoralShooterLaunch() {
    mortar = RobotContainer.mortar;

    // Do not use mortar in requirements, this would cancel the Ramp-up Command!!!
    // addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_LAUNCH");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT_LAUNCH);

    Dashboard.setCoralState(GamePieceState.LAUNCHING);

    mortar.startLauncher(CoralShooterConstants.launcherShootSetpoint);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mortar.isLoaded()) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dashboard.setCoralState(GamePieceState.IDLE);
    if (interrupted) {
      mortar.stopShooter();
    }
    mortar.stopLauncher();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CoralShooterConstants.launchTimer.in(Seconds));
  }
}
