package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndDeAlgae extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public ToPathAndDeAlgae() {
    System.out.printf("[Cmd: %s]", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getAlgaePreset().getPose();

    addCommands(
        new FollowPathToPose(poseSupplier).onlyIf(() -> PresetManager.usingDtp()),
        new ToDeAlgae(() -> PresetManager.getAlgaePreset()));
  }
}
