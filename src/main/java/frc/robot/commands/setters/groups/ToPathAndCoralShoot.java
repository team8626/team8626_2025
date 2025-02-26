package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndCoralShoot extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public ToPathAndCoralShoot() {
    System.out.printf("[Cmd: %s]", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();

    addCommands(
        new FollowPathToPose(poseSupplier).onlyIf(() -> PresetManager.usingDtp()),
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds)
            .onlyIf(() -> PresetManager.usingDtp()),
        new ToCoralShoot());
  }
}
