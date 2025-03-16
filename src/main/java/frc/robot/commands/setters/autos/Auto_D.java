package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.commands.setters.groups.ToCoralShoot3;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_D extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_D() {
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.D, CoralLevel.L4, 0);
    addCommands(new DriveToPoseFinkle(poseSupplier), new ToCoralShoot3());
  }
}
