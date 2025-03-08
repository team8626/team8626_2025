package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_PickupLeft extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_PickupLeft() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();
    poseSupplier = () -> PresetManager.getRobotPoseFromPickupSide(PickupSide.LEFT);

    addCommands(
        new PrintCommand("X: " + poseSupplier.get().getX() + " Y: " + poseSupplier.get().getY()),
        new FollowPathToPose(poseSupplier),
        // new DriveFinalApproach(poseSupplier),
        // TODO: UNCOMMENT THIS AND TEST
        new ToCoralIntake());
  }
}
