// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class AlgaeShooterRampUp extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  private AngularVelocity desiredRPM = Presets.ALGAE_FLOOR.getRPM();
  private final AngularVelocity RPMTolerance = AlgaeShooterConstants.shooterRPMTolerance;
  private boolean overrideRPM = false;
  private AngularVelocity dashboardRPM = RPM.of(0);
  private boolean doNotStopOnInterrupt = false;

  public AlgaeShooterRampUp(Supplier<AngularVelocity> newRPM) {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/leftAtSetPoint", false);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/rightAtSetPoint", false);

    this.desiredRPM = newRPM.get();
    this.setTAGString("ALGAESHOOTER_RAMPUP");
  }

  public AlgaeShooterRampUp withDoNotStopOnInterrupt() {
    doNotStopOnInterrupt = true;
    return this;
  }

  @Override
  public void initialize() {
    printf("RPM: %f", desiredRPM.in(RPM));

    Dashboard.setAlgaeState(GamePieceState.RAMPING_UP);

    overrideRPM = SmartDashboard.getBoolean("Commands/AlgaeShooterRampUp/OverrideRPM", false);
    dashboardRPM = RPM.of(SmartDashboard.getNumber("Commands/AlgaeShooterRampUp/ForcedRMP", 0));

    if (overrideRPM) {
      desiredRPM = dashboardRPM;
    }
    algae501.startRampUp(desiredRPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted && !doNotStopOnInterrupt) {
      algae501.stopAll();
    }
  }

  @Override
  public boolean isFinished() {
    boolean leftAtSetpoint = false;
    boolean rightAtSetpoint = false;

    AngularVelocity currentRPMLeft = algae501.getShooterRPMLeft();
    AngularVelocity currentRPMRight = algae501.getShooterRPMRight();

    leftAtSetpoint =
        Math.abs(Math.abs(currentRPMLeft.in(RPM)) - desiredRPM.in(RPM)) <= RPMTolerance.in(RPM);
    rightAtSetpoint =
        Math.abs(Math.abs(currentRPMRight.in(RPM)) - desiredRPM.in(RPM)) <= RPMTolerance.in(RPM);

    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/leftAtSetPoint", leftAtSetpoint);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/rightAtSetPoint", rightAtSetpoint);
    return leftAtSetpoint && rightAtSetpoint;
  }
}
