package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndDeAlgaefy extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<AlgaePreset> presetSupplier;

  public ToPathAndDeAlgaefy() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getAlgaePreset().getPose();
    presetSupplier = () -> PresetManager.getAlgaePreset();

    addCommands(
        new ConditionalCommand(
            new FollowPathToPose(poseSupplier, () -> 12), // 12 inches Offset
            new InstantCommand(),
            () -> PresetManager.usingDtp()),
        new ToDeAlgaefy(presetSupplier));
  }
}
