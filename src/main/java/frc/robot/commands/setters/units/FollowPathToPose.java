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
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FollowPathToPose extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private CS_DriveSubsystem drivebase;

  private Pose2d targetPose;
  private PathConstraints constraints;
  private Command pathfindingCommand;
  private double offsetInches = 0;

  private boolean hasValidPose = false;

  public FollowPathToPose(Supplier<Pose2d> desiredPoseSupplier) {
    this(desiredPoseSupplier, () -> 0);
  }

  public FollowPathToPose(Supplier<Pose2d> desiredPoseSupplier, DoubleSupplier newOffsetInches) {
    drivebase = RobotContainer.drivebase;
    this.poseSupplier = desiredPoseSupplier;
    this.offsetInches = newOffsetInches.getAsDouble();

    addRequirements(drivebase);
    this.setTAGString("FOLLOWPATHTOPOSE");
  }

  @Override
  public void initialize() {

    if ((this.poseSupplier != null) && (this.poseSupplier.get() != null)) {
      hasValidPose = true;
    }

    if (hasValidPose) {

      targetPose = poseSupplier.get();

      Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);

      println(
          "X: "
              + targetPose.getX()
              + " Y: "
              + targetPose.getY()
              + " Offset: "
              + offsetInches
              + "in");

      Pose2d offsetPose =
          targetPose.plus(
              new Transform2d(Units.inchesToMeters(this.offsetInches), 0, new Rotation2d()));

      constraints =
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      // constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited

      pathfindingCommand = AutoBuilder.pathfindToPose(offsetPose, constraints, 0.0);
      pathfindingCommand.schedule();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    Commodore.setCommodoreState(CommodoreState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return hasValidPose && pathfindingCommand.isFinished();
  }
}
