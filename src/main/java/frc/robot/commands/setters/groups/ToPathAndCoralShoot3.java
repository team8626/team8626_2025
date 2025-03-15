package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndCoralShoot3 extends SequentialCommandGroup {
  private Supplier<Pose2d> poseSupplier;
  private Supplier<CoralPreset> presetSupplier;

  public ToPathAndCoralShoot3() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();
    presetSupplier = () -> PresetManager.getCoralPreset();

    addCommands(
        new PrintCommand("ToPathAndCoralShoot3 : Using DTP ")
            .onlyIf(() -> PresetManager.usingDtp()),
        new PrintCommand("ToPathAndCoralShoot3 : NOT Using DTP ")
            .onlyIf(() -> !PresetManager.usingDtp()),
        // new FollowPathToPose(poseSupplier, () -> 24).onlyIf(() -> PresetManager.usingDtp()),
        new DriveToPoseFinkle(poseSupplier, true).onlyIf(() -> PresetManager.usingDtp()),
        new ToCoralShoot3());
  }
}
