package frc.robot.commands.setters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.commands.setters.units.DriveToPose;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class Auto_L extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<Pose2d> finalPoseSupplier;

  public Auto_L() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.L, CoralLevel.L4, 10);
    finalPoseSupplier = () -> PresetManager.getRobotPoseFromTarget(CoralBranch.L, CoralLevel.L4, 0);

    addCommands(
        // new FollowPathToPose(poseSupplier),
        new PrintCommand("I am Driving !!!!!!!!!!!!!!!!!!!!!!!!"),
        new DriveToPose(finalPoseSupplier),
        // new WaitCommand(CoralShooterConstants.shooterTimerSeconds),
        new PrintCommand("I am Shooting !!!!!!!!!!!!!!!!!!!!!!!!"));
    // new ToCoralShoot2());
  }
}
