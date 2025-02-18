// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterIntake extends CS_Command {
  private CoralShooterSubsystem mortar;
  private final Timer timer = new Timer();

  public CoralShooterIntake() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_INTAKE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.INTAKING);
    mortar.startIntake();
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mortar.isLoaded()) {
      timer.start();
    }
    return timer.hasElapsed(CoralShooterConstants.launchTimerSeconds);
  }
}
