// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.Presets;

public class CoralShooterRampUp2 extends CS_Command {
  private CoralShooterSubsystem mortar;

  private double desiredRpmLeft;
  private double desiredRpmRight;
  private final double RPMTolerance = CoralShooterConstants.RPMTolerance;
  private final double RPMDifferentialTolerance = CoralShooterConstants.RPMDifferentialTolerance;

  public CoralShooterRampUp2() {
    mortar = RobotContainer.mortar;

    desiredRpmLeft = Presets.CORAL_L4.RPMLeft;
    desiredRpmRight = Presets.CORAL_L4.RPMRight;

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP2");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);
    // printf("[CORALSHOOTER_RAMPUP] RPM Left: %f, RPM Right: %f", desiredRPMLeft, desiredRPMRight);
    // mortar.startRampUp(desiredRPMLeft, desiredRPMRight);
    desiredRpmLeft =
        SmartDashboard.getNumber(
            "Subsystem/PresetManager/CoralPreset/RPMLeft", Presets.CORAL_L4.RPMLeft);
    desiredRpmRight =
        SmartDashboard.getNumber(
            "Subsystem/PresetManager/CoralPreset/RPMRight", Presets.CORAL_L4.RPMRight);
    printf("[CORALSHOOTER_RAMPUP] RPM Left: %f, RPM Right: %f", desiredRpmLeft, desiredRpmRight);
    mortar.startRampUp(desiredRpmLeft, desiredRpmRight);
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

    // atSetpoint =
    //     Math.abs(Math.abs(currentRPMLeft) - desiredRPMLeft) <= RPMTolerance
    //         && Math.abs(Math.abs(currentRPMRight) - desiredRPMRight) <= RPMTolerance;
    atSetpoint =
        Math.abs(Math.abs(currentRPMLeft) - desiredRpmLeft) <= RPMTolerance
            && Math.abs(Math.abs(currentRPMRight) - desiredRpmRight) <= RPMTolerance;
    return atSetpoint;
  }
}
