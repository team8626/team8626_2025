package frc.robot.commands.setters.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.UIConstants.PICKUP_SIDE;
import frc.robot.commands.setters.units.CoralShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_PickupRight extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private CoralShooterSubsystem mortar = RobotContainer.mortar;

  public Auto_PickupRight() {
    poseSupplier = () -> PresetManager.getRobotPoseFromPickupSide(() -> PICKUP_SIDE.RIGHT);
    addCommands(
        new DriveToPoseFinkle2(poseSupplier, () -> Inches.of(1), () -> Degrees.of(5)),
        // new WaitUntilCommand(() -> mortar.hasCoral() || mortar.isLoaded()),
        new CoralShooterIntake().onlyIf(() -> !mortar.isLoaded()));
  }
}
