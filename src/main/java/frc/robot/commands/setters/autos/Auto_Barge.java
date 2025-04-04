package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.groups.ToAlgaeShoot;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class Auto_Barge extends SequentialCommandGroup {

  private Supplier<Pose2d> targetPose;

  public Auto_Barge() {
    addCommands(
        new ToAlgaeShoot(() -> Presets.ALGAE_HIGH),
        new DriveToPoseFinkle2(targetPose)
            .deadlineFor(new AlgaeShooterIntake(() -> Presets.ALGAE_HIGH.getRPM())));
  }
}
