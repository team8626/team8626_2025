package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.DriveRotateToAngle;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

/**
 * @deprecated This class is obsolete and scheduled for removal.
 */
@Deprecated
public class ToDriveAndCoralShoot extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public ToDriveAndCoralShoot() {
    System.out.printf("[Cmd: %s]", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();

    addCommands(
        new ParallelCommandGroup(
            new DriveRotateToAngle(poseSupplier), new FollowPathToPose(poseSupplier)),
        new ToCoralShoot());
  }
}
