// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.CoralShooterIntake;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndCoralIntake extends SequentialCommandGroup {
  private Supplier<Pose2d> targetPose =
      () -> PresetManager.getRobotPoseFromPickupSide(PresetManager.getPickupSide());

  public ToPathAndCoralIntake() {
    CoralShooterSubsystem mortar = RobotContainer.mortar;

    addCommands(
        // Drive to Target Pose
        new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                AutoBuilder.pathfindToPose(targetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.25),
                Commodore.getSetStateCommand(CommodoreState.IDLE))
            .onlyIf(() -> !targetPose.get().equals(new Pose2d()) && !mortar.isLoaded())
            .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE)),

        // Coral Intake
        new CoralShooterIntake().onlyIf(() -> !mortar.isLoaded()),
        Commodore.getSetStateCommand(CommodoreState.IDLE));
  }
}
