package frc.robot.commands.setters.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UIConstants.PICKUP_SIDE;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_PickupLeft extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_PickupLeft() {
    poseSupplier = () -> PresetManager.getRobotPoseFromPickupSide(() -> PICKUP_SIDE.LEFT);
    addCommands(
        new DriveToPoseFinkle2(poseSupplier, () -> Inches.of(1), () -> Degrees.of(5)),
        new ToCoralIntake());
  }
}
