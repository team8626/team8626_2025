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
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import java.util.function.DoubleSupplier;

public class AlgaeShooterRampUp extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  private double desiredRPM = AlgaeShooterConstants.shootRPM;
  private final double RPMTolerance = AlgaeShooterConstants.shooterRPMTolerance;
  private boolean overrideRPM = false;
  private double dashboardRPM = 0;

  public AlgaeShooterRampUp(DoubleSupplier newRPM) {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/leftAtSetPoint", false);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/rightAtSetPoint", false);

    this.desiredRPM = newRPM.getAsDouble();
    this.setTAGString("ALGAESHOOTER_RAMPUP");
  }

  @Override
  public void initialize() {
    printf("RPM: %f", desiredRPM);

    Dashboard.setAlgaeState(GamePieceState.RAMPING_UP);

    overrideRPM = SmartDashboard.getBoolean("Commands/AlgaeShooterRampUp/OverrideRPM", false);
    dashboardRPM = SmartDashboard.getNumber("Commands/AlgaeShooterRampUp/ForcedRMP", 0);

    if (overrideRPM) {
      desiredRPM = dashboardRPM;
    }
    algae501.startRampUp(desiredRPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      algae501.stopAll();
    }
  }

  @Override
  public boolean isFinished() {
    boolean leftAtSetpoint = false;
    boolean rightAtSetpoint = false;

    double currentRPMLeft = algae501.getShooterRPMLeft();
    double currentRPMRight = algae501.getShooterRPMRight();

    leftAtSetpoint = Math.abs(Math.abs(currentRPMLeft) - desiredRPM) <= RPMTolerance;
    rightAtSetpoint = Math.abs(Math.abs(currentRPMRight) - desiredRPM) <= RPMTolerance;

    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/leftAtSetPoint", leftAtSetpoint);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/rightAtSetPoint", rightAtSetpoint);
    return leftAtSetpoint && rightAtSetpoint;
  }
}
