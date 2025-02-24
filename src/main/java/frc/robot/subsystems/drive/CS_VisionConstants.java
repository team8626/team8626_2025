package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class CS_VisionConstants {

  /** Vision Suppport - Team 8626 */
  //   public static final String kATBackRightCameraName = "Arducam_AT001";
  //   public static final Transform3d kATRobotToBackRightCam =
  //       new Transform3d(
  //           new Translation3d(-0.294805, -0.280851, 0.237332),
  //           new Rotation3d(26.033445, Units.degreesToRadians(-20),
  // Units.degreesToRadians(143.308536)));

  //   public static final String kATBackLeftCameraName = "Arducam_AT002";
  //   public static final Transform3d kATRobotToBackLeftCam =
  //       new Transform3d(
  //         new Translation3d(-0.294805, 0.280851, 0.237332),
  //         new Rotation3d(-26.033445, Units.degreesToRadians(-20),
  // Units.degreesToRadians(216.691464)));

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // // The standard deviations of our vision estimated poses, which affect correction rate
  // // (Fake values. Experiment and determine estimation noise on an actual robot.)
  // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  enum CS_Cameras {
    Arducam_AT001(
        "Arducam_AT001",
        new Rotation3d(26.033445, Units.degreesToRadians(-20), Units.degreesToRadians(143.308536)),
        new Translation3d(-0.294805, -0.280851, 0.237332),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)),

    Arducam_AT002(
        "Arducam_AT002",
        new Rotation3d(-26.033445, Units.degreesToRadians(-20), Units.degreesToRadians(216.691464)),
        new Translation3d(-0.294805, 0.280851, 0.237332),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));

    private final String name;
    private final Rotation3d rotation;
    private final Translation3d translation;
    private Transform3d cameraToRobot;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;

    private CS_Cameras(
        String name,
        Rotation3d rotation,
        Translation3d translation,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevsMatrix) {
      this.name = name;
      this.rotation = rotation;
      this.translation = translation;
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
      this.cameraToRobot = new Transform3d(translation, rotation);
    }

    public String getName() {
      return name;
    }

    public Transform3d getCameraToRobot() {
      return cameraToRobot;
    }
  }
}
