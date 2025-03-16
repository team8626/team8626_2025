package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_PickupLeft extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_PickupLeft() {
    poseSupplier = () -> PresetManager.getRobotPoseFromPickupSide(PickupSide.LEFT);
    addCommands(new DriveToPoseFinkle(poseSupplier), new ToCoralIntake());
  }
}
