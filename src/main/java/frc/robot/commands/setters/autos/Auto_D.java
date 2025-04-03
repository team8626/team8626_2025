package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UIConstants.CORAL_BRANCH;
import frc.robot.UIConstants.CORAL_LEVEL;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class Auto_D extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<CoralPreset> preset = () -> Presets.CORAL_L4;

  public Auto_D() {
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CORAL_BRANCH.D, CORAL_LEVEL.L4, 0);
    addCommands(
        new DriveToPoseFinkle2(poseSupplier, () -> preset.get().getOffset()), new ToCoralShoot());
  }
}
