// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import java.util.function.Supplier;

public class AlgaeShooterIntake extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  private Timer timer = new Timer();
  private boolean algaeDetected = false;
  private Supplier<AngularVelocity> desiredRPM;
  private boolean doNoStopOnIntake = false;

  public AlgaeShooterIntake() {
    algae501 = RobotContainer.algae501;
    desiredRPM = () -> AlgaeShooterConstants.intakeRPM;

    addRequirements(algae501);

    this.setTAGString("ALGAESHOOTER_INTAKE");
  }

  public AlgaeShooterIntake(Supplier<AngularVelocity> newSpeed) {
    algae501 = RobotContainer.algae501;
    desiredRPM = newSpeed;

    addRequirements(algae501);

    this.setTAGString("ALGAESHOOTER_INTAKE");
  }

  public AlgaeShooterIntake withDoNotStopOnIntake() {
    doNoStopOnIntake = true;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    algaeDetected = false;

    Commodore.setCommodoreState(CommodoreState.ALGAE_INTAKE);
    Dashboard.setAlgaeState(GamePieceState.INTAKING);
    algae501.startIntake(desiredRPM.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (algae501.isLoaded()) {
      Dashboard.setAlgaeState(GamePieceState.LOADED);
    } else {
      Dashboard.setAlgaeState(GamePieceState.IDLE);
    }
    if (!doNoStopOnIntake) algae501.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((algae501.isLoaded()) && (!algaeDetected)) {
      timer.start();
      algaeDetected = true;
    }
    return timer.hasElapsed(AlgaeShooterConstants.intakeTimerSeconds);
  }
}
