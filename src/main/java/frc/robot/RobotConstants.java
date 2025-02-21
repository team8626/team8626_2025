package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class RobotConstants {

  // Robot types
  public enum RobotType {
    SIMBOT, /* Simulation Robot*/
    KITBOT, /* Kitbot running on WC Drive*/
    DART, /* 2024 Competition Robor*/
    DEVBOT, /* 2025 Development Robot*/
    COMPBOT /* Placeholder for 2025 Competition Robot*/;
  }

  // ****************************************************************************************
  // Change robot type here if needed
  // Possible options: SIMBOT, KITBOT, DART, DEVBOT, COMPBOT
  //
  public static RobotType robotType = RobotType.DEVBOT;
  public static boolean tracesEnabled = true;
  public static boolean tuningEnabled = false; // DO NOT COMMIT WHILE ENABLED

  public static boolean disableHAL = false;

  // Subsystems positions/translations for simulation in AdvantageScope
  // public static Translation2d intakeOffset = new Translation2d(Units.inchesToMeters(24),
  // Units.inchesToMeters(0));

  /** Defines the robot's physical dimensions. */
  public static double FrameWidthInches = 27;

  public static double FrameLengthInches = 27;
  public static double bumperThicknessInches = 3;

  //   public static double robotCenterToEdge = Units.inchesToMeters(FrameLength + bumperThickness /
  // 2);

  private static double robotCenterOffsetX = FrameLengthInches / 2.0 + bumperThicknessInches;
  private static double robotCenterOffsetY = FrameWidthInches / 2.0 + bumperThicknessInches;
  private static double robotCenterOffsetTheta = 0;

  public static Pose2d robotCenterOffset =
      new Pose2d(
          Units.inchesToMeters(robotCenterOffsetX),
          Units.inchesToMeters(robotCenterOffsetY),
          Rotation2d.fromDegrees(robotCenterOffsetTheta));

  public static Pose3d mastPoseOffset =
      new Pose3d(
          0,
          0,
          Units.inchesToMeters(5),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));
  public static Pose3d stage2PoseOffset =
      new Pose3d(
          0,
          0,
          Units.inchesToMeters(6),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));
  public static Pose3d stage3PoseOffset =
      new Pose3d(
          0,
          0,
          Units.inchesToMeters(5),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));
  public static Pose3d carriagePoseOffset =
      new Pose3d(
          0,
          0,
          Units.inchesToMeters(4),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));
  public static Pose3d wristPoseOffset =
      new Pose3d(
          Units.inchesToMeters(2.5),
          Units.inchesToMeters(-0.5),
          Units.inchesToMeters(5.5),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));
  public static Pose3d shooterPoseOffset =
      new Pose3d(
          Units.inchesToMeters(2.5 + 6.5),
          Units.inchesToMeters(-0.5),
          Units.inchesToMeters(5.5 + 2.626886),
          new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90)));

  // /** Defines position of the cameras on the robot. */
  //   public static class Vision {
  //     // April tag camera constants
  //     public static final String kATBackCameraName = "Arducam_AT002";
  //     public static final Transform3d kATRobotToBackCam =
  //         new Transform3d(
  //             new Translation3d(-0.222275, -0.276124, 0.251296),
  //             new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(167.04746)));

  /** Defines position of the cameras on the robot. */
  // public static class Vision_2024 { // TODO: Can be removed?
  //   // April tag camera constants
  //   public static final String kATBackCameraName = "Arducam_AT002";
  //   public static final Transform3d kATRobotToBackCam =
  //       new Transform3d(
  //           new Translation3d(-0.222275, -0.276124, 0.251296),
  //           new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(167.04746)));

  //     public static final String kATFrontCameraName = "Arducam_AT001";
  //     public static final Transform3d kATRobotToFrontCam =
  //         new Transform3d(
  //             new Translation3d(0.197621, 0.005613, 0.504500),
  //             new Rotation3d(0, Units.degreesToRadians(-19.562757), Units.degreesToRadians(17)));

  //     // The layout of the AprilTags on the field
  //     public static final AprilTagFieldLayout kTagLayout =
  //         AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  //   public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  //   public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  // }

  // ****************************************************************************************
  // UI Constants
  public static class UIConstants {
    public static final List<String> allowedCoralLevels = Arrays.asList("L1", "L2", "L3", "L4");
    public static final List<String> allowedIntakeSides = Arrays.asList("LEFT", "RIGHT");
    public static final List<String> allowedCoralBranches =
        Arrays.asList("A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L");
    public static final List<String> allowedAlgaePositions =
        Arrays.asList("AB", "CD", "EF", "GH", "IJ", "KL");
  }
}
