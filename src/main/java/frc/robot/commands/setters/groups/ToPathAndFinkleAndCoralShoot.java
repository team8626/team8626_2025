// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndFinkleAndCoralShoot extends SequentialCommandGroup {

  Supplier<Pose2d> targetPose = () -> PresetManager.getCoralPreset().get().getPose();
  Supplier<Pose2d> offsetPose =
      () ->
          PresetManager.getCoralPreset()
              .get()
              .getPose()
              .plus(new Transform2d(Units.inchesToMeters(36), 0, new Rotation2d()));
  Supplier<CoralPreset> coralPreset = PresetManager.getCoralPreset();

  public ToPathAndFinkleAndCoralShoot() {
    addCommands(
        // Drive to Offset Pose
        new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                AutoBuilder.pathfindToPose(offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 1))
            .onlyIf(() -> !targetPose.get().equals(new Pose2d()))
            .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE)),

        // Drive to Target Pose
        new DriveToPoseFinkle(targetPose).onlyIf(() -> !targetPose.get().equals(new Pose2d())),

        // Coral Shoot
        new ToCoralShoot(coralPreset));
  }
}
