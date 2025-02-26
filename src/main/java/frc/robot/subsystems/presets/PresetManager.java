// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.presets.AlgaePreset.AlgaeLevelPreset;
import frc.robot.subsystems.presets.AlgaePreset.AlgaePosition;
import java.util.Optional;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;

public class PresetManager extends CS_SubsystemBase {
  private String uiSelectedCoralLevel = "";
  private String uiSelectedIntakeSide = "";
  private String uiSelectedCoralBranch = "";
  private String uiSelectedAlgaeFace = "";

  private String uiCurrentCoralLevel = "L4";
  private String uiCurrentIntakeSide = "----";
  private String uiCurrentCoralBranch = "-";
  private String uiCurrentAlgaeFace = "--";
  private static boolean uiCurrentUseDtp = true;

  private final StringSubscriber coralBranchSub =
      NetworkTableInstance.getDefault()
          .getStringTopic("SmartDashboard/Presets/UI/SelectedCoralBranch")
          .subscribe("-");
  private final StringSubscriber coralLevelSub =
      NetworkTableInstance.getDefault()
          .getStringTopic("SmartDashboard/Presets/UI/SelectedCoralLevel")
          .subscribe("L4");
  private final StringSubscriber algaeFaceSub =
      NetworkTableInstance.getDefault()
          .getStringTopic("SmartDashboard/Presets/UI/SelectedAlgaeFace")
          .subscribe("--");
  private final StringSubscriber intakeSideSub =
      NetworkTableInstance.getDefault()
          .getStringTopic("SmartDashboard/Presets/UI/SelectedIntakeSide")
          .subscribe("----");
  private final BooleanSubscriber dtpEnabledSub =
      NetworkTableInstance.getDefault()
          .getBooleanTopic("SmartDashboard/Presets/UI/SelectedDTPEnabled")
          .subscribe(true);

  // private CoralPreset autoCoralPreset = new CoralPreset( "autoPreset", new Pose2d(), 0.0, false);

  private static CoralPreset currentCoralPreset =
      new CoralPreset("autoPreset", new Pose2d(), 0.0, 0.0, false);
  private static AlgaePreset currentAlgaePreset =
      new AlgaePreset("autoPreset", new Pose2d(), AlgaeLevelPreset.FLOOR, false);

  // Singleton instance
  private static PresetManager instance;

  // Public method to provide access to the singleton instance
  public static PresetManager getInstance() {
    if (instance == null) {
      instance = new PresetManager();
    }
    return instance;
  }

  private PresetManager() {
    super();
  }

  @Override
  public void CS_periodic() {
    updateUIData();
    updatePreset();
  }

  // Return the current presets
  public static CoralPreset getCoralPreset() {
    // TODO: Return null if not ready or have an isCoralPresetReady() method
    return currentCoralPreset;
  }

  public static AlgaePreset getAlgaePreset() {
    // TODO: Return null if not ready or have an isAlgaePresetReady() method
    return currentAlgaePreset;
  }

  public static boolean usingDtp() {
    return uiCurrentUseDtp;
  }

  public static boolean isCoralPresetReady() {
    return currentCoralPreset.isReady();
  }

  public static boolean isAlgaePresetReady() {
    return currentAlgaePreset.isReady();
  }

  //   public static Preset getAimAndShootPreset(Pose2d robotPose) {

  //     // angleAdjust =
  //     //     SmartDashboard.getNumber("Presets/AimPreset/Launch Angle Adjust (deg)",
  // angleAdjust);
  //     // ooomf = SmartDashboard.getNumber("Presets/AimPreset/Ooomf", ooomf);
  //     // launchRPMTopMultiplier =
  //     //     SmartDashboard.getNumber("Presets/AimPreset/TopRPMMultiplier",
  // launchRPMTopMultiplier);

  //     double maxArmRotationInsideFrame = 201; /* Degrees */
  //     double targetX =
  // AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getX();
  //     double targetY =
  // AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getY();

  //     double robotX = robotPose.getX();
  //     double robotY = robotPose.getY();
  //     double z0 = Units.inchesToMeters(20); // Shooting Height

  //     double targetHeight = Units.inchesToMeters(80.5);
  //     double targetDistance =
  //         Math.sqrt(
  //             (targetX - robotX) * (targetX - robotX) + (targetY - robotY) * (targetY - robotY));

  //     Rotation2d targetElevation =
  //         new Rotation2d(Math.atan((FieldConstants.topRightSpeaker.getZ() / targetDistance)));

  //     Rotation2d targetRotation =
  //         AllianceFlipUtil.apply(
  //             new Rotation2d(Math.acos((targetY - robotY) / targetDistance) - Math.PI / 2));

  //     Rotation2d robotRotation = AllianceFlipUtil.apply(targetRotation);

  //     // double ooomf = 0; // m.s-1

  //     double vZ = Math.sqrt((m_ooomf * m_ooomf) + (targetHeight - z0) * 2 * 9.81);
  //     double tm = (vZ - m_ooomf) / 9.81;
  //     double vX = targetDistance / tm;

  //     // double angleAdjust = -9;
  //     // double launchRPMTopMultiplier = 0.6;

  //     Rotation2d launchAngle = new Rotation2d(Math.atan(vZ / vX));
  //     Rotation2d armAngle =
  //         new Rotation2d(
  //             Units.degreesToRadians(
  //                 launchAngle.getDegrees()
  //                     + 180
  //                     - 30 /* Horizontal arm: 180deg, Shooter/Arm: -30deg */
  //                     + m_angleAdjust)); /* Adjust after tuning */

  //     if (armAngle.getDegrees() > maxArmRotationInsideFrame) {
  //       armAngle = new Rotation2d(Units.degreesToRadians(maxArmRotationInsideFrame));
  //     }

  //     double launchVelocity = Math.sqrt((vX * vX) + (vZ * vZ));
  //     double launchRPM =
  //         Math.min(
  //             (launchVelocity / (Math.PI * /*ShooterConstants.kFlywheelDiameterMeters / 2*/1)) *
  // 60,
  //             /*ShooterConstants.kMaxRPM*/0);

  //     SmartDashboard.putNumber("Presets/AimPreset/Robot X", robotX);
  //     SmartDashboard.putNumber("Presets/AimPreset/Robot Y", robotY);
  //     SmartDashboard.putNumber("Presets/AimPreset/Shooter Z (z0)", z0);

  //     SmartDashboard.putNumber("Presets/AimPreset/Target X", targetX);
  //     SmartDashboard.putNumber("Presets/AimPreset/Target Y", targetY);
  //     SmartDashboard.putNumber("Presets/AimPreset/Target Z", targetHeight);

  //     SmartDashboard.putNumber("Presets/AimPreset/v0", targetHeight);
  //     SmartDashboard.putNumber("Presets/AimPreset/Target Z", targetHeight);

  //     SmartDashboard.putNumber("Presets/AimPreset/Target Distance (m)", targetDistance);

  //     SmartDashboard.putNumber("Presets/AimPreset/vX", vX);
  //     SmartDashboard.putNumber("Presets/AimPreset/vZ", vZ);
  //     SmartDashboard.putNumber("Presets/AimPreset/tM", tm);

  //     SmartDashboard.putNumber(
  //         "Presets/AimPreset/Target Rotation (deg)", targetRotation.getDegrees());
  //     SmartDashboard.putNumber(
  //         "Presets/AimPreset/Target Elevation (deg)", targetElevation.getDegrees());
  //     SmartDashboard.putNumber("Presets/AimPreset/Launch Angle (deg)", launchAngle.getDegrees());
  //     SmartDashboard.putNumber("Presets/AimPreset/Launch Velocity (m.s-1)", launchVelocity);
  //     SmartDashboard.putNumber("Presets/AimPreset/Launch Speed (RPM)", launchRPM);
  //     SmartDashboard.putNumber("Presets/AimPreset/Launch Arm Angle (deg)",
  // armAngle.getDegrees());
  //     SmartDashboard.putNumber(
  //         "Presets/AimPreset/Robot Rotation Angle (deg)", robotRotation.getDegrees());

  //     return new Preset(
  //         "AUTO AIM",
  //         armAngle.getDegrees(),
  //         0.0,
  //         (int) (launchRPM * m_launchRPMTopMultiplier),
  //         (int) launchRPM,
  //         new Pose2d(robotX, robotY, robotRotation));
  // return null;
  // }

  @Override
  public void initDashboard() {
    println("Initializing PresetManager Dashboard");

    SmartDashboard.putString("Presets/UI/SelectedCoralLevel", "");
    SmartDashboard.putString("Presets/UI/SelectedCoralBranch", "");
    SmartDashboard.putString("Presets/UI/SelectedAlgaeFace", "");
    SmartDashboard.putString("Presets/UI/SelectedIntakeSide", "");

    SmartDashboard.putStringArray(
        "Presets/UI/AllowedCoralLevels", UIConstants.allowedCoralLevels.toArray(new String[0]));
    SmartDashboard.putStringArray(
        "Presets/UI/AllowedCoralBranches", UIConstants.allowedCoralBranches.toArray(new String[0]));
    SmartDashboard.putStringArray(
        "Presets/UI/allowedAlgaePositions",
        UIConstants.allowedAlgaePositions.toArray(new String[0]));
  }

  @Override
  public void updateDashboard() {}

  public void updateUIData() {
    // Set Alliance Color
    // TODO Move this to the Dashboard subsystem
    String alliance = "UNKNOWN";
    if (DriverStation.isFMSAttached()) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          alliance = "RED";
        }
        if (ally.get() == Alliance.Blue) {
          alliance = "BLUE";
        }
      }
    }

    SmartDashboard.putString("Presets/UI/AllianceColor", alliance);

    // Get Values from UI
    this.uiSelectedCoralBranch = coralBranchSub.get();
    this.uiSelectedCoralLevel = coralLevelSub.get();
    this.uiSelectedAlgaeFace = algaeFaceSub.get();
    this.uiSelectedIntakeSide = intakeSideSub.get();
    this.uiCurrentUseDtp = dtpEnabledSub.get();
  }

  // Update the Presets based on the UI Data
  private void updatePreset() {
    // New Coral Level or Branch
    // Compute the new position ands shooting parameters
    if ((!uiSelectedCoralLevel.equals(this.uiCurrentCoralLevel)
            && UIConstants.allowedCoralLevels.contains(uiSelectedCoralLevel))
        || (!uiSelectedCoralBranch.equals(this.uiCurrentCoralBranch)
            && UIConstants.allowedCoralBranches.contains(uiSelectedCoralBranch))) {

      printf("Updating Presets - %s, %s", uiSelectedCoralBranch, uiSelectedCoralLevel);

      // New Selected Value for CORAL (Branch or Level)
      this.uiCurrentCoralBranch = uiSelectedCoralBranch;
      this.uiCurrentCoralLevel = uiSelectedCoralLevel;

      Pose2d branchPose =
          getBranchPoseFromTarget(this.uiCurrentCoralBranch, this.uiCurrentCoralLevel);

      Pose2d robotPose =
          branchPose.plus(
              new Transform2d(
                  RobotConstants.robotCenterOffset.getX()
                      + Units.inchesToMeters(1.625), //  1.625 for Branch inset to face distance...
                  0,
                  branchPose.getRotation()));
      printf(
          "New Coral Pose - x: %3f, y: %3f, theta: %3f",
          robotPose.getX(), robotPose.getY(), branchPose.getRotation().getDegrees());

      currentCoralPreset.setPose(
          AllianceFlipUtil.apply(
              new Pose2d(robotPose.getX(), robotPose.getY(), branchPose.getRotation())));

      currentCoralPreset.setLevelPreset(CoralLevelPreset.valueOf(uiCurrentCoralLevel));
      currentCoralPreset.setReady(true);
    } else {
      // No new data or no valid data
    }

    // New Algae Position
    // Compute the new position ands shooting parameters
    if (!uiSelectedAlgaeFace.equals(this.uiCurrentAlgaeFace)
        && UIConstants.allowedAlgaePositions.contains(uiSelectedAlgaeFace)) {

      printf("Updating Algae Presets - %s", uiSelectedAlgaeFace);

      this.uiCurrentAlgaeFace = uiSelectedAlgaeFace;

      Pose2d facePose = getFacePoseFromTarget(this.uiCurrentAlgaeFace);
      Pose2d robotPose =
          facePose
              .plus(
                  new Transform2d(
                      RobotConstants.robotCenterOffset.getX(), 0, (facePose.getRotation())))
              .rotateBy(new Rotation2d(Math.PI));
      Pose2d goodRobotPose =
          new Pose2d(
              robotPose.getX(),
              robotPose.getY(),
              new Rotation2d(facePose.getRotation().getRadians()));

      printf(
          "New Algae Pose - x: %3f, y: %3f, theta: %3f",
          goodRobotPose.getX(), goodRobotPose.getY(), goodRobotPose.getRotation().getDegrees());

      currentAlgaePreset.setPose(
          AllianceFlipUtil.apply(
              new Pose2d(goodRobotPose.getX(), goodRobotPose.getY(), facePose.getRotation())));

      currentAlgaePreset.setPosition(AlgaePosition.valueOf(uiSelectedAlgaeFace));
      currentAlgaePreset.setReady(true);
    } else {
      // No new data or no valid data
    }
  }

  private static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions.get(objective.branchId()).get(objective.reefLevel()).toPose2d();
  }

  private Pose2d getBranchPoseFromTarget(String branch, String level) {
    ReefLevel branchHeight;
    int branchId = 0;

    switch (branch) {
      case "L1":
        branchHeight = ReefLevel.fromLevel(1);
        break;
      case "L2":
        branchHeight = ReefLevel.fromLevel(2);
        break;
      case "L3":
        branchHeight = ReefLevel.fromLevel(3);
        break;
      case "4":
      default:
        branchHeight = ReefLevel.fromLevel(4);
    }

    switch (branch) {
      case "A":
        branchId = 1;
        break;
      case "B":
        branchId = 0;
        break;
      case "C":
        branchId = 11;
        break;
      case "D":
        branchId = 10;
        break;
      case "E":
        branchId = 9;
        break;
      case "F":
        branchId = 8;
        break;
      case "G":
        branchId = 7;
        break;
      case "H":
        branchId = 6;
        break;
      case "I":
        branchId = 5;
        break;
      case "J":
        branchId = 4;
        break;
      case "K":
        branchId = 3;
        break;
      case "L":
        branchId = 2;
        break;

      default:
        branchId = 0;
        break;
    }
    return Reef.branchPositions.get(branchId).get(branchHeight).toPose2d();
  }

  private Pose2d getFacePoseFromTarget(String face) {
    int faceId = 0;
    boolean isLow = false;

    switch (face) {
      case "AB":
        faceId = 0;
        isLow = false;
        break;
      case "CD":
        faceId = 5;
        isLow = true;
        break;
      case "EF":
        faceId = 4;
        isLow = false;
        break;
      case "GH":
        faceId = 3;
        isLow = true;
        break;
      case "IJ":
        faceId = 2;
        isLow = false;
        break;
      case "KL":
        faceId = 1;
        isLow = true;
        break;
    }

    printf(
        "New Face (%s - %s) - x: %3f, y: %3f, theta: %3f",
        face,
        isLow ? "Low" : "High",
        Reef.centerFaces[faceId].getX(),
        Reef.centerFaces[faceId].getY(),
        Reef.centerFaces[faceId].getRotation().getDegrees());

    return Reef.centerFaces[faceId].rotateBy(new Rotation2d(Math.PI));
  }
}
