// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

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

public class Tune_AlgaeShooter extends CS_Command {
  private AlgaeShooterSubsystem algae501;
  private AngularVelocity desiredRPM = AlgaeShooterConstants.shootRPM;

  public Tune_AlgaeShooter() {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);

    this.desiredRPM = Presets.ALGAE_NETFROM10FT.getRPM();
    this.setTAGString("ALGAESHOOTER_RAMPUP");
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngularVelocity newRPM =
        RPM.of(
            SmartDashboard.getNumber(
                "Subsystem/AlgaeShooter/Shooting RPM", AlgaeShooterConstants.shootRPM.in(RPM)));
    desiredRPM = newRPM;
    printf("RPM: %f", desiredRPM);

    Dashboard.setAlgaeState(GamePieceState.RAMPING_UP);
    algae501.startRampUp(desiredRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      algae501.stopAll();
      Dashboard.setAlgaeState(GamePieceState.IDLE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
