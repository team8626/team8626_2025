package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.commands.setters.groups.ToCoralShoot3;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_I extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<Pose2d> finalPoseSupplier;

  public Auto_I() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.I, CoralLevel.L4, 0);

    addCommands(
        new FollowPathToPose(poseSupplier),
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds),
        new ToCoralShoot3());
  }
}
