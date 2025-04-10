package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
    COMPBOT /* 2025 Competition Robot*/;
  }

  // ****************************************************************************************
  // Change robot type here if needed
  // Possible options: SIMBOT, KITBOT, DART, DEVBOT, COMPBOT
  //
  public static RobotType robotType = RobotType.COMPBOT;
  public static boolean tracesEnabled = true;
  public static boolean tuningEnabled = false; // DO NOT COMMIT WHILE ENABLED
  public static boolean disableHAL = false;

  // PathFinding Constants
  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(6.0, 12.0, 2 * Math.PI, 4 * Math.PI);

  // Subsystems positions/translations for simulation in AdvantageScope
  // public static Translation2d intakeOffset = new Translation2d(Units.inchesToMeters(24),
  // Units.inchesToMeters(0));

  /** Defines the robot's physical dimensions. */
  public static double FrameWidthInches = 27;

  public static double FrameLengthInches = 27;

  public static final double driveBaseRadius =
      Math.hypot(FrameWidthInches / 2, FrameLengthInches / 2);
  public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(6.77);

  //  Slower robot speed for demonstrations
  public static final Double MAX_LINEAR_VELOCITY_LOW = 0.2;

  public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(720);

  public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(1.5);
  public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
      DegreesPerSecondPerSecond.of(120);

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
}
