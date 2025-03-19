package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.UIConstants.PICKUP_SIDE;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_PickupRight extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_PickupRight() {
    poseSupplier = () -> PresetManager.getRobotPoseFromPickupSide(() -> PICKUP_SIDE.RIGHT);
    addCommands(new DriveToPoseFinkle(poseSupplier, () -> 1, () -> 5), new ToCoralIntake());
  }
}
