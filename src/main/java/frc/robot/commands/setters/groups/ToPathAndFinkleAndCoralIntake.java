// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.CoralShooterIntake;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToPathAndFinkleAndCoralIntake extends SequentialCommandGroup {
  private Supplier<Pose2d> targetPose =
      () -> PresetManager.getRobotPoseFromPickupSide(PresetManager.getPickupSide());

  private Supplier<Pose2d> offsetPose =
      () -> targetPose.get().plus(new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d()));

  public ToPathAndFinkleAndCoralIntake() {
    CoralShooterSubsystem mortar = RobotContainer.mortar;

    addCommands(
        // Drive to Target Pose
        new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                AutoBuilder.pathfindToPose(offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.25),
                Commodore.getSetStateCommand(CommodoreState.IDLE))
            .onlyIf(() -> !targetPose.get().equals(new Pose2d()) && !mortar.isLoaded())
            .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE)),

        // Drive to Target Pose
        new DriveToPoseFinkle2(targetPose, () -> Inches.of(1), () -> Degrees.of(5))
            .onlyIf(() -> !targetPose.get().equals(new Pose2d()))
            .withDeadline(new CoralShooterIntake()),

        // Coral Intake
        new CoralShooterIntake().onlyIf(() -> !mortar.isLoaded()),
        Commodore.getSetStateCommand(CommodoreState.IDLE));
  }
}
