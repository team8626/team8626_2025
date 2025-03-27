package frc.robot.vizualization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CS_SubsystemBase;

public class Visualization extends CS_SubsystemBase {
  Pose3d robotPose = new Pose3d();
  Pose3d mastPose = new Pose3d();
  Pose3d stage2Pose = new Pose3d();
  Pose3d stage3Pose = new Pose3d();
  Pose3d carriagePose = new Pose3d();
  Pose3d wristPose = new Pose3d();
  Pose3d shooterPose = new Pose3d();

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> arrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  // Singleton instance
  private static Visualization instance;

  // Public method to provide access to the singleton instance
  public static Visualization getInstance() {
    if (instance == null) {
      instance = new Visualization();
    }
    return instance;
  }

  private Visualization() {
    super();
  }

  @Override
  public void CS_periodic() {
    robotPose = new Pose3d(RobotContainer.drivebase.getPose());

    mastPose = RobotConstants.mastPoseOffset;
    stage2Pose =
        RobotConstants.stage2PoseOffset.plus(
            new Transform3d(
                0, 0, -(RobotContainer.elevator.getHeight().in(Meters) * 0.33), new Rotation3d()));
    stage3Pose =
        RobotConstants.stage3PoseOffset.plus(
            new Transform3d(
                0, 0, -(RobotContainer.elevator.getHeight().in(Meters) * 0.67), new Rotation3d()));
    carriagePose =
        RobotConstants.carriagePoseOffset.plus(
            new Transform3d(
                0, 0, -(RobotContainer.elevator.getHeight().in(Meters)), new Rotation3d()));
    wristPose =
        RobotConstants.wristPoseOffset.plus(
            new Transform3d(
                0, 0, -(RobotContainer.elevator.getHeight().in(Meters)), new Rotation3d()));
    shooterPose =
        RobotConstants.shooterPoseOffset.plus(
            new Transform3d(
                0,
                0,
                -(RobotContainer.elevator.getHeight().in(Meters)),
                new Rotation3d((RobotContainer.wrist.getAngle().in(Degrees) - 180), 0, 0)));

    publisher.set(robotPose);
    arrayPublisher.set(
        new Pose3d[] {mastPose, stage2Pose, stage3Pose, carriagePose, wristPose, shooterPose});
  }

  // private final DoublePublisher robotXPublisher;
  // private final DoublePublisher robotYPublisher;
  // private final DoublePublisher robotAnglePublisher;
  // private final IntegerPublisher robotStatusPublisher;

  // public Visualization() {
  //     var networkTableInstance = NetworkTableInstance.getDefault();
  //     var robotTable = networkTableInstance.getTable("Robot");

  //     robotXPublisher = robotTable.getDoubleTopic("robotX").publish();
  //     robotYPublisher = robotTable.getDoubleTopic("robotY").publish();
  //     robotAnglePublisher = robotTable.getDoubleTopic("robotAngle").publish();
  //     robotStatusPublisher = robotTable.getIntegerTopic("robotStatus").publish();
  // }

  // public void updateRobotPosition(double x, double y, double angle) {
  //     robotXPublisher.set(x);
  //     robotYPublisher.set(y);
  //     robotAnglePublisher.set(angle);
  // }

  // public void updateRobotStatus(int status) {
  //     robotStatusPublisher.set(status);
  // }
}
