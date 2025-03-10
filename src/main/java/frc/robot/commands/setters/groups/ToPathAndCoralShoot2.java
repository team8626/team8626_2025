package frc.robot.commands.setters.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.setters.units.FollowPathToPose;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndCoralShoot2 extends SequentialCommandGroup {

  private Supplier<Pose2d> poseSupplier;
  private Supplier<CoralPreset> presetSupplier;

  public ToPathAndCoralShoot2() {
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());
    poseSupplier = () -> PresetManager.getCoralPreset().getPose();
    presetSupplier = () -> PresetManager.getCoralPreset();

    addCommands(
        new PrintCommand("ToPathAndCoralShoot : Using DTP ").onlyIf(() -> PresetManager.usingDtp()),
        new PrintCommand("ToPathAndCoralShoot : NOT Using DTP ")
            .onlyIf(() -> !PresetManager.usingDtp()),
        new FollowPathToPose(poseSupplier).onlyIf(() -> PresetManager.usingDtp()),
        // new DriveFinalApproach(poseSupplier),
        // TODO: UNCOMMENT THIS AND TEST
        new WaitCommand(CoralShooterConstants.shooterTimerSeconds)
            .onlyIf(() -> PresetManager.usingDtp()),
        new ToCoralShoot2());
  }
}
