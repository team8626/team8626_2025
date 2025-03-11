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
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CoralShooterRampUp3 extends CS_Command {
  private CoralShooterSubsystem mortar;

  private DoubleSupplier desiredRpmLeftSupplier;
  private DoubleSupplier desiredRpmRightSupplier;
  private final double RPMTolerance = CoralShooterConstants.RPMTolerance;
  private final double RPMDifferentialTolerance = CoralShooterConstants.RPMDifferentialTolerance;
  private String coralPresetName = "";

  public CoralShooterRampUp3() {
    mortar = RobotContainer.mortar;

    desiredRpmLeftSupplier = () -> PresetManager.getCoralPreset().RPMLeft;
    desiredRpmRightSupplier = () -> PresetManager.getCoralPreset().RPMRight;
    coralPresetName = "From PresetManager";

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  public CoralShooterRampUp3(Supplier<CoralPreset> newPreset) {
    mortar = RobotContainer.mortar;

    desiredRpmLeftSupplier = () -> newPreset.get().RPMLeft;
    desiredRpmRightSupplier = () -> newPreset.get().RPMRight;
    coralPresetName = "From Supplier - " + newPreset.get().name;

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  public CoralShooterRampUp3(DoubleSupplier newRPMLeft, DoubleSupplier newRPMRight) {
    mortar = RobotContainer.mortar;

    desiredRpmLeftSupplier = newRPMLeft;
    desiredRpmRightSupplier = newRPMRight;
    coralPresetName =
        "From DoubleSuppliers"
            + desiredRpmLeftSupplier.getAsDouble()
            + " / "
            + desiredRpmRightSupplier.getAsDouble();

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);
    printf(
        "(%s) RPM Left: %f, RPM Right: %f",
        coralPresetName,
        desiredRpmLeftSupplier.getAsDouble(),
        desiredRpmRightSupplier.getAsDouble());
    mortar.startRampUp(desiredRpmLeftSupplier.getAsDouble(), desiredRpmRightSupplier.getAsDouble());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    Dashboard.setCoralState(GamePieceState.IDLE);
    if (interrupted) {
      mortar.stopAll();
    }
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    atSetpoint =
        Math.abs(Math.abs(currentRPMLeft) - desiredRpmLeftSupplier.getAsDouble()) <= RPMTolerance
            && Math.abs(Math.abs(currentRPMRight) - desiredRpmRightSupplier.getAsDouble())
                <= RPMTolerance;

    return atSetpoint;
  }
}
