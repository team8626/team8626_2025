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

public class CoralShooterIntake extends CS_Command {
  private CoralShooterSubsystem mortar;
  private final Timer timer = new Timer();
  private final Timer intakeTimer = new Timer();
  private boolean intakePaused = false;

  public CoralShooterIntake() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_INTAKE");
  }

  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.CORAL_INTAKE);
    Dashboard.setCoralState(GamePieceState.INTAKING);
    mortar.startIntake();
    intakeTimer.stop();
    intakeTimer.reset();
    intakeTimer.start();

    timer.stop();
    timer.reset();
  }

  @Override
  public void execute() {
    if (intakePaused) {
      if (intakeTimer.hasElapsed(0.25)) {
        mortar.startIntake();
        intakePaused = false;
        intakeTimer.reset();
      }
    } else {
      if (intakeTimer.hasElapsed(10)) {
        mortar.stopAll();
        intakePaused = true;
        intakeTimer.reset();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (mortar.isLoaded()) {
      Dashboard.setCoralState(GamePieceState.LOADED);
    } else {
      Dashboard.setCoralState(GamePieceState.IDLE);
    }
    mortar.stopAll();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    if (mortar.isLoaded()) {
      timer.start();
    }
    return timer.hasElapsed(CoralShooterConstants.launchTimer.in(Seconds));
  }
}
