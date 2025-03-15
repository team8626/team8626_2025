// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterIntake2 extends CS_Command {
  private CoralShooterSubsystem mortar;
  private final Timer timer = new Timer();
  private final Timer intakeTimer = new Timer();
  private boolean intakePaused = false;

  public CoralShooterIntake2() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_INTAKE_2");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.INTAKING);
    mortar.startIntake();
    intakeTimer.stop();
    intakeTimer.reset();
    intakeTimer.start();

    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakePaused) {
      if (intakeTimer.hasElapsed(0.25)) {
        mortar.startIntake();
        intakePaused = false;
        intakeTimer.reset();
      }
    } else {
      if (intakeTimer.hasElapsed(3.0)) {
        mortar.stopAll();
        intakePaused = true;
        intakeTimer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mortar.isLoaded()) {
      Dashboard.setCoralState(GamePieceState.LOADED);
      new RumbleCommand().schedule();

    } else {
      Dashboard.setCoralState(GamePieceState.IDLE);
    }
    mortar.stopAll();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mortar.isLoaded()) {
      timer.start();
    }
    return timer.hasElapsed(CoralShooterConstants.launchTimerSeconds);
  }
}
