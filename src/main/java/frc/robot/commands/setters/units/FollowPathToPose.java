package frc.robot.commands.setters.units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.Supplier;

public class FollowPathToPose extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private CS_DriveSubsystem drivebase;

  private Pose2d targetPose;
  private PathConstraints constraints;
  private Command pathfindingCommand;
  private boolean hasPose = false;

  public FollowPathToPose(Supplier<Pose2d> poseSupplier) {
    drivebase = RobotContainer.drivebase;
    this.poseSupplier = poseSupplier;

    // addRequirements(drive);

    this.setTAGString("FOLLOWPATHTOPOSE");
  }

  @Override
  public void initialize() {

    targetPose = poseSupplier.get();

    if (this.targetPose != null) {
      println("X: " + targetPose.getX() + " Y: " + targetPose.getY());

      constraints =
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      // constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited

      pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
      pathfindingCommand.schedule();
      hasPose = true;
    } else {
      hasPose = false;
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hasPose && pathfindingCommand.isFinished();
  }
}
