package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.commands.setters.groups.ToCoralShootL1Only;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_G extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<Pose2d> finalPoseSupplier;

  public Auto_G() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.G, CoralLevel.L4, 10);
    finalPoseSupplier =
        () -> PresetManager.getRobotPoseFromTarget(CoralBranch.G, CoralLevel.L4, -2);

    addCommands(
        new FollowPathToPose(finalPoseSupplier),
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds),
        new ToCoralShootL1Only());
  }
}
