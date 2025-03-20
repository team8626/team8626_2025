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
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CoralShooterRampUp extends CS_Command {
  private CoralShooterSubsystem mortar;
  private boolean overrideRPM = false;
  private double dashboardRPMLeft = 0;
  private double dashboardRPMRight = 0;
  private DoubleSupplier desiredRpmLeft;
  private DoubleSupplier desiredRpmRight;
  private final double RPMTolerance = CoralShooterConstants.RPMTolerance;
  private String coralPresetName = "";

  public CoralShooterRampUp() {
    this(PresetManager.getCoralPreset());
  }

  public CoralShooterRampUp(Supplier<CoralPreset> newPreset) {
    mortar = RobotContainer.mortar;

    desiredRpmLeft = () -> newPreset.get().RPMLeft;
    desiredRpmRight = () -> newPreset.get().RPMRight;
    coralPresetName = "From Supplier - " + newPreset.get().name;

    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/leftAtSetPoint", false);
    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/rightAtSetPoint", false);
    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  public CoralShooterRampUp(DoubleSupplier newRPMLeft, DoubleSupplier newRPMRight) {
    mortar = RobotContainer.mortar;

    desiredRpmLeft = newRPMLeft;
    desiredRpmRight = newRPMRight;
    coralPresetName =
        "From DoubleSuppliers"
            + desiredRpmLeft.getAsDouble()
            + " / "
            + desiredRpmRight.getAsDouble();

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);

    overrideRPM = SmartDashboard.getBoolean("Commands/CoralShooterRampUp/OverrideRPM", false);
    dashboardRPMLeft = SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPLeft", 0);
    dashboardRPMRight = SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPRight", 0);

    if (overrideRPM) {
      desiredRpmLeft = () -> dashboardRPMLeft;
      desiredRpmRight = () -> dashboardRPMRight;
    }

    printf(
        "(%s) RPM Left: %f, RPM Right: %f",
        coralPresetName, desiredRpmLeft.getAsDouble(), desiredRpmRight.getAsDouble());
    mortar.startRampUp(desiredRpmLeft.getAsDouble(), desiredRpmRight.getAsDouble());
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
    boolean leftAtSetpoint = false;
    boolean rightAtSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    leftAtSetpoint =
        Math.abs(Math.abs(currentRPMLeft) - desiredRpmLeft.getAsDouble()) <= RPMTolerance;
    rightAtSetpoint =
        Math.abs(Math.abs(currentRPMRight) - desiredRpmRight.getAsDouble()) <= RPMTolerance;

    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/leftAtSetPoint", leftAtSetpoint);
    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/rightAtSetPoint", rightAtSetpoint);

    return leftAtSetpoint && rightAtSetpoint;
  }
}
