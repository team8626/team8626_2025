package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UIConstants.CORAL_BRANCH;
import frc.robot.UIConstants.CORAL_LEVEL;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_L extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_L() {
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CORAL_BRANCH.L, CORAL_LEVEL.L4, 0);
    addCommands(new DriveToPoseFinkle(poseSupplier), new ToCoralShoot());
  }
}
