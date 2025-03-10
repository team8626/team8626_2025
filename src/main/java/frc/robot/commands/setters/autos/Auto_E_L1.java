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

public class Auto_E_L1 extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;

  public Auto_E_L1() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.E, CoralLevel.L1, 0);

    addCommands(
        new FollowPathToPose(poseSupplier),
        // new DriveFinalApproach(poseSupplier),
        // TODO: UNCOMMENT THIS AND TEST
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds),
        new ToCoralShootL1Only());
  }
}
