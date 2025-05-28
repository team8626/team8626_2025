package frc.robot.commands.setters.groups;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToClosestBargeAndAlgaeShoot extends SequentialCommandGroup {

  private Supplier<AlgaePreset> algaePreset;
  private Supplier<Pose2d> targetPose = () -> algaePreset.get().getShootingPose();

  private Supplier<Pose2d> shootPose =
      () ->
          new Pose2d(
              new Translation2d(
                  targetPose.get().getX(),
                  Math.max(targetPose.get().getY(), RobotContainer.drivebase.getPose().getY())),
              targetPose.get().getRotation());

  private Supplier<Pose2d> offsetPose =
      () ->
          shootPose
              .get()
              .plus(
                  new Transform2d(
                      Units.inchesToMeters(-12),
                      Units.inchesToMeters(0),
                      new Rotation2d(Units.degreesToRadians(0))));

  public ToClosestBargeAndAlgaeShoot(Supplier<AlgaePreset> presetSupplier) {
    algaePreset = presetSupplier;

    addCommands(
        new SequentialCommandGroup(
            // Drive to Offset Pose
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                AutoBuilder.pathfindToPose(offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.5)
                    .onlyIf(() -> !targetPose.get().equals(new Pose2d()))
                    .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE))),

            // Drive to Target Pose while setting subsystems
            new ParallelCommandGroup(
                // Drive to Target Pose
                new DriveToPoseFinkle2(shootPose, () -> Inches.of(1), () -> Degrees.of(3))
                    .onlyIf(() -> !targetPose.get().equals(new Pose2d())),

                // Adjust Subsystems & Start Rampup
                new ToSubsystemsPreset(algaePreset)),

            // Shoot Algae
            new ToAlgaeShoot(algaePreset),

            // Reset subsystems
            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
            Commodore.getSetStateCommand(CommodoreState.IDLE)));
  }
}
