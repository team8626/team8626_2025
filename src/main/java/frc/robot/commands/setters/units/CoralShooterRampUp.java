// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class CoralShooterRampUp extends CS_Command {
  private CoralShooterSubsystem mortar;
  private boolean overrideRPM = false;
  private AngularVelocity dashboardRPMLeft = RPM.of(0);
  private AngularVelocity dashboardRPMRight = RPM.of(0);
  private Supplier<AngularVelocity> desiredRpmLeft;
  private Supplier<AngularVelocity> desiredRpmRight;
  private final AngularVelocity RPMTolerance = CoralShooterConstants.RPMTolerance;
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

  public CoralShooterRampUp(
      Supplier<AngularVelocity> newRPMLeft, Supplier<AngularVelocity> newRPMRight) {
    mortar = RobotContainer.mortar;

    desiredRpmLeft = newRPMLeft;
    desiredRpmRight = newRPMRight;
    coralPresetName =
        "From DoubleSuppliers"
            + desiredRpmLeft.get().in(RPM)
            + " / "
            + desiredRpmRight.get().in(RPM);

    addRequirements(mortar);
    this.setTAGString("CORALSHOOTER_RAMPUP3");
  }

  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT_RAMPUP);
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);

    overrideRPM = SmartDashboard.getBoolean("Commands/CoralShooterRampUp/OverrideRPM", false);
    dashboardRPMLeft =
        RPM.of(SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPLeft", 0));
    dashboardRPMRight =
        RPM.of(SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPRight", 0));

    if (overrideRPM) {
      desiredRpmLeft = () -> dashboardRPMLeft;
      desiredRpmRight = () -> dashboardRPMRight;
    }

    printf(
        "(%s) RPM Left: %f, RPM Right: %f",
        coralPresetName, desiredRpmLeft.get().in(RPM), desiredRpmRight.get().in(RPM));
    mortar.startRampUp(desiredRpmLeft.get(), desiredRpmRight.get());
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

    AngularVelocity currentRPMLeft = mortar.getShooterRPMLeft();
    AngularVelocity currentRPMRight = mortar.getShooterRPMRight();

    leftAtSetpoint =
        Math.abs(Math.abs(currentRPMLeft.in(RPM)) - desiredRpmLeft.get().in(RPM))
            <= RPMTolerance.in(RPM);
    rightAtSetpoint =
        Math.abs(Math.abs(currentRPMRight.in(RPM)) - desiredRpmRight.get().in(RPM))
            <= RPMTolerance.in(RPM);

    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/leftAtSetPoint", leftAtSetpoint);
    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/rightAtSetPoint", rightAtSetpoint);

    return leftAtSetpoint && rightAtSetpoint;
  }
}
