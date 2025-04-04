package frc.robot.commands.setters.units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FollowPathToPose extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private SwerveSubsystem drivebase;

  private Pose2d targetPose;
  private Pose2d offsetPose;
  private PathConstraints constraints;
  private Command pathfindingCommand;
  private double offsetInches = 0;
  private double endSpeed = 0;

  private boolean hasValidPose = false;

  public FollowPathToPose(Supplier<Pose2d> desiredPoseSupplier) {
    this(desiredPoseSupplier, () -> 0);
  }

  public FollowPathToPose(Supplier<Pose2d> desiredPoseSupplier, DoubleSupplier newOffsetInches) {
    this(desiredPoseSupplier, newOffsetInches, () -> 0.0);
  }

  /**
   * Follows a path to a desired pose.
   *
   * @param desiredPoseSupplier The desired pose to follow
   * @param newOffsetInches The offset from the desired pose (inches)
   * @param newEndSpeed The speed at the end of the path (m.s^-1)
   */
  public FollowPathToPose(
      Supplier<Pose2d> desiredPoseSupplier,
      DoubleSupplier newOffsetInches,
      DoubleSupplier newEndSpeed) {

    drivebase = RobotContainer.drivebase;

    this.poseSupplier = desiredPoseSupplier;
    this.offsetInches = newOffsetInches.getAsDouble();
    this.endSpeed = newEndSpeed.getAsDouble();

    addRequirements(drivebase);
    this.setTAGString("FOLLOWPATHTOPOSE");
  }

  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);

    hasValidPose = false;

    if ((this.poseSupplier != null) && (this.poseSupplier.get() != null)) {
      hasValidPose = true;
    }

    if (hasValidPose) {
      println("--------- Starting Pathfinding ---------");

      targetPose = poseSupplier.get();

      println(
          "X: "
              + targetPose.getX()
              + " Y: "
              + targetPose.getY()
              + " Offset: "
              + offsetInches
              + "in");

      offsetPose =
          targetPose.plus(
              new Transform2d(Units.inchesToMeters(this.offsetInches), 0, new Rotation2d()));

      println("With Offset X: " + offsetPose.getX() + " Y: " + offsetPose.getY());
      constraints =
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      // constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited

      pathfindingCommand = AutoBuilder.pathfindToPose(offsetPose, constraints, this.endSpeed);
      pathfindingCommand.schedule();
    }
  }

  @Override
  public void execute() {
    Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO_INTERRUPTED);
    } else {

      Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO_FINISHED);
    }
  }

  @Override
  public boolean isFinished() {
    boolean retVal = false;
    if (hasValidPose
        && ((Math.abs(offsetPose.getX() - drivebase.getPose().getX()) < 0.10)
            && (Math.abs(offsetPose.getY() - drivebase.getPose().getY()) < 0.10))) {
      retVal = true;
    } else if (!hasValidPose) {
      retVal = true;
    }
    return retVal;
  }
}
