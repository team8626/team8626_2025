// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import java.util.function.DoubleSupplier;

public class CoralShooterRampUp extends CS_Command {
  private CoralShooterSubsystem mortar;

  private double desiredRPMLeft;
  private double desiredRPMRight;
  private final double RPMTolerance = CoralShooterConstants.RPMTolerance;
  private final double RPMDifferentialTolerance = CoralShooterConstants.RPMDifferentialTolerance;

  public CoralShooterRampUp() {}

  // public CoralShooterRampUp(DoubleSupplier new_RPMLeft, DoubleSupplier new_RPMRight) {
  //   mortar = RobotContainer.mortar;

  //   desiredRPMLeft = new_RPMLeft.getAsDouble();
  //   desiredRPMRight = new_RPMRight.getAsDouble();

  //   addRequirements(mortar);
  //   this.setTAGString("CORALSHOOTER_RAMPUP");
  // }

  public CoralShooterRampUp(
      DoubleSupplier new_RPMLeftSupplier, DoubleSupplier new_RPMRightSupplier) {
    mortar = RobotContainer.mortar;

    desiredRPMLeft = new_RPMLeftSupplier.getAsDouble();
    desiredRPMRight = new_RPMRightSupplier.getAsDouble();

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    printf("[CORALSHOOTER_RAMPUP] RPM Left: %f, RPM Right: %f", desiredRPMLeft, desiredRPMRight);
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);
    mortar.startRampUp(desiredRPMLeft, desiredRPMRight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      mortar.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    atSetpoint =
        Math.abs(Math.abs(currentRPMLeft) - desiredRPMLeft) <= RPMTolerance
            && Math.abs(Math.abs(currentRPMRight) - desiredRPMRight) <= RPMTolerance;
    return atSetpoint;
  }
}
