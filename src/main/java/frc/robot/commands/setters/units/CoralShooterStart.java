// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.utils.CS_Utils;

public class CoralShooterStart extends CS_Command {
  private CoralShooterSubsystem mortar;

  private double desiredRPM = CoralShooterConstants.shooterRPM;
  private final double RPMTolerance = CoralShooterConstants.shooterRPMTolerance;

  public CoralShooterStart() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_START");

    // Initialize SmartDashboard
    SmartDashboard.putNumber("Commands/ShooterStart/DesiredRPM", 1234);
    SmartDashboard.putNumber("Commands/ShooterStart/CurrentRPM LEFT", 1234);
    SmartDashboard.putNumber("Commands/ShooterStart/CurrentRPM RIGHT", 1234);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get values from SmartDashboard when the command is initialized
    double newRPM = SmartDashboard.getNumber("Commands/ShooterStart/DesiredRPM", 5678);
    desiredRPM = CS_Utils.updateFromSmartDashboard(newRPM, desiredRPM, (value) -> mortar.setShooterRPM(value));

    mortar.setShooterRPM(desiredRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get values from SmartDashboard periodically
    double newRPM = SmartDashboard.getNumber("Commands/ShooterStart/DesiredRPM", 5678);
    desiredRPM = CS_Utils.updateFromSmartDashboard(newRPM, desiredRPM, (value) -> mortar.setShooterRPM(value));

    // Update SmartDashboard
    SmartDashboard.putNumber("Commands/ShooterStart/DesiredRPM", 1234);
    SmartDashboard.putNumber("Commands/ShooterStart/CurrentRPM LEFT", 1234);
    SmartDashboard.putNumber("Commands/ShooterStart/CurrentRPM RIGHT", 1234);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mortar.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    atSetpoint = Math.abs(currentRPMLeft - desiredRPM) <= RPMTolerance && Math.abs(currentRPMRight - desiredRPM) <= RPMTolerance;

    return atSetpoint;
  }
}
