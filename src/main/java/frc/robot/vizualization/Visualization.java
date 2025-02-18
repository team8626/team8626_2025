package frc.robot.vizualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CS_SubsystemBase;

public class Visualization extends CS_SubsystemBase {
  Pose3d robotPose = new Pose3d();
  Pose3d mastPose = new Pose3d(0, 0, 0, new Rotation3d());
  Pose3d stage2Pose = new Pose3d(0, 0, 0, new Rotation3d());
  Pose3d stage3Pose = new Pose3d(0, 0, 0, new Rotation3d());
  Pose3d carriagePose = new Pose3d(0, 0, 0, new Rotation3d());
  Pose3d wristPose = new Pose3d(0, 0, 0, new Rotation3d());
  Pose3d shooterPose = new Pose3d(0, 0, 0, new Rotation3d());

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> arrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  @Override
  public void CS_periodic() {
    robotPose = RobotContainer.drivebase.getPose3d();

    mastPose = RobotConstants.mastPoseOffset;
    stage2Pose = RobotConstants.stage2PoseOffset;
    stage3Pose = RobotConstants.stage3PoseOffset;
    carriagePose = RobotConstants.carriagePoseOffset;
    wristPose = RobotConstants.wristPoseOffset;
    shooterPose = RobotConstants.shooterPoseOffset;

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
