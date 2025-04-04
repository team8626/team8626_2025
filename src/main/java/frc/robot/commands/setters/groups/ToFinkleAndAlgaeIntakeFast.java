package frc.robot.commands.setters.groups;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToFinkleAndAlgaeIntakeFast extends SequentialCommandGroup {

  private AlgaeShooterSubsystem algae501 = RobotContainer.algae501;
  private Supplier<Pose2d> targetPose =
      () -> PresetManager.getAlgaePreset().get().getDealgaefyPose();
  private Supplier<Pose2d> offsetPose =
      () ->
          PresetManager.getAlgaePreset()
              .get()
              .getDealgaefyPose()
              .plus(new Transform2d(Units.inchesToMeters(-12), 0, new Rotation2d()));

  private Supplier<AlgaePreset> algaePreset = PresetManager.getAlgaePreset();

  public ToFinkleAndAlgaeIntakeFast() {
    addCommands(
        new SequentialCommandGroup(
            // Adjust Subsystems
            new ToSubsystemsPreset(algaePreset)
                .raceWith(
                    new AlgaeShooterIntake(() -> algaePreset.get().getRPM())
                        .withDoNotStopOnIntake()),

            // Drive to Target Pose
            new DriveToPoseFinkle2(targetPose, () -> Inches.of(2), () -> Degrees.of(5))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d())),

            // Algae Intake
            new AlgaeShooterIntake(() -> algaePreset.get().getRPM())
                .withDoNotStopOnIntake()
                .onlyIf(() -> !algae501.isLoaded())));
  }
}
