package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UIConstants.ALGAE_FACE;
import frc.robot.commands.setters.groups.ToSubsystemsPreset;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class Auto_5 extends SequentialCommandGroup {

  private Supplier<Pose2d> targetPose;

  public Auto_5() {
    targetPose = () -> PresetManager.getRobotPoseFromAlgae(ALGAE_FACE.IJ, 0);
    addCommands(
        new ToSubsystemsPreset(() -> Presets.ALGAE_HIGH),
        new DriveToPoseFinkle2(targetPose)
            .deadlineFor(new AlgaeShooterIntake(() -> Presets.ALGAE_HIGH.getRPM())));
  }
}
