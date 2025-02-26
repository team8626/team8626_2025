package frc.robot.commands.setters.units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.Supplier;

/// TODO: UNUSED - Can be removed
public class DriveToPose extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private CS_DriveSubsystem drivebase;

  Pose2d targetPose;
  PathConstraints constraints;
  Command pathfindingCommand;

  public DriveToPose(Supplier<Pose2d> poseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    drivebase = RobotContainer.drivebase;
    this.poseSupplier = poseSupplier;

    // addRequirements(drive);

    this.setTAGString("DRIVETOPOSE");
  }

  @Override
  public void initialize() {
    // drive.driveToPose(poseSupplier).schedule();
    targetPose = poseSupplier.get();

    // Create the constraints to use while pathfinding
    constraints =
        new PathConstraints(5.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(360));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    pathfindingCommand.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
