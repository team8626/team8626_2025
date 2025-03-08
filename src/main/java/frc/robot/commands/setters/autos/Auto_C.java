package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.commands.setters.groups.ToCoralShoot2;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_C extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_C() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.C, CoralLevel.L4);

    addCommands(
        new FollowPathToPose(poseSupplier),
        // new DriveFinalApproach(poseSupplier),
        // TODO: UNCOMMENT THIS AND TEST
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds),
        new ToCoralShoot2());
  }
}
