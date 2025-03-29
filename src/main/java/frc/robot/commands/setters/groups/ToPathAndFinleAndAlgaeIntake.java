package frc.robot.commands.setters.groups;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToPathAndFinleAndAlgaeIntake extends SequentialCommandGroup {

  private Supplier<Pose2d> targetPose =
      () -> PresetManager.getAlgaePreset().get().getDealgaefyPose();
  Supplier<Pose2d> offsetPose =
      () ->
          PresetManager.getAlgaePreset()
              .get()
              .getDealgaefyPose()
              .plus(new Transform2d(Units.inchesToMeters(-12), 0, new Rotation2d()));

  private Supplier<AlgaePreset> algaePreset = PresetManager.getAlgaePreset();

  public ToPathAndFinleAndAlgaeIntake() {
    addCommands(
        new SequentialCommandGroup(
            // Drive to Target Pose
            new SequentialCommandGroup(
                    Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                    AutoBuilder.pathfindToPose(
                        offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d()))
                .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE)),

            // Adjust Subsystems
            new ToSubsystemsPreset(algaePreset),

            // Drive to Target Pose
            new DriveToPoseFinkle2(targetPose, () -> Inches.of(2), () -> Degrees.of(5))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d())),

            // Algae Intake
            new AlgaeShooterIntake(() -> algaePreset.get().getRPM()),

            // Drive Away
            new DriveToPoseFinkle2(offsetPose, () -> Inches.of(6), () -> Degrees.of(10))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d())),

            // Stow
            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
            Commodore.getSetStateCommand(CommodoreState.IDLE)));
  }
}
