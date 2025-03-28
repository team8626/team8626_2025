package frc.robot.commands.setters.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UIConstants.CORAL_BRANCH;
import frc.robot.UIConstants.CORAL_LEVEL;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_G extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_G() {
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CORAL_BRANCH.G, CORAL_LEVEL.L4, 0);
    addCommands(new DriveToPoseFinkle2(poseSupplier, () -> Inches.of(7)), new ToCoralShoot());
  }
}
