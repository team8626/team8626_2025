// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.RobotConstants.UIConstants.AlgaeFace2;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.RobotConstants.UIConstants.DTP;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.objectivetracker.ReefControlsIOServer;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;

public class PresetManager extends CS_SubsystemBase {
  private String uiSelectedCoralLevel = "";
  private String uiSelectedIntakeSide = "";
  private String uiSelectedCoralBranch = "";
  private String uiSelectedAlgaeFace = "";

  private UIConstants.CoralLevel uiSelectedCoralLevel2 = UIConstants.defaultCoralLevel;
  private UIConstants.PickupSide uiSelectedIntakeSide2 = UIConstants.defaultPickupSide;
  private UIConstants.CoralBranch uiSelectedCoralBranch2 = CoralBranch.NONE;
  private UIConstants.AlgaeFace2 uiSelectedAlgaeFace2 = AlgaeFace2.NONE;
  private UIConstants.DTP uiSelectedDtp2 = UIConstants.defaultDTP;

  private UIConstants.CoralLevel uiCurrentCoralLevel2 = UIConstants.defaultCoralLevel;
  private UIConstants.PickupSide uiCurrentIntakeSide2 = UIConstants.defaultPickupSide;
  private UIConstants.CoralBranch uiCurrentCoralBranch2 = CoralBranch.NONE;
  private UIConstants.AlgaeFace2 uiCurrentAlgaeFace2 = AlgaeFace2.NONE;
  private UIConstants.DTP uiCurrentDtp2 = UIConstants.defaultDTP;

  private String uiCurrentCoralLevel = "L4";
  private String uiCurrentIntakeSide = "----";
  private String uiCurrentCoralBranch = "-";
  private String uiCurrentAlgaeFace = "--";

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

  // private CoralPreset autoCoralPreset = new CoralPreset( "autoPreset", new Pose2d(), 0.0, false);

  private static CoralPreset currentCoralPreset =
      new CoralPreset("autoPreset", new Pose2d(), 0.0, 0.0, false);
  private static AlgaePreset currentAlgaePreset =
      new AlgaePreset("autoPreset", new Pose2d(), 0.0, 0.0, 0.0, false);

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
    return currentCoralPreset;
  }

  public static AlgaePreset getAlgaePreset() {
    return currentAlgaePreset;
  }

  // private StructPublisher<Pose2d> robotPostPublisher =
  //     NetworkTableInstance.getDefault()
  //         .getStructTopic("SmartDashboard/Presets/Pose2d", Pose2d.struct)
  //         .publish();

  //   private static double m_ooomf = 3; // m.s-1
  //   private static double m_angleAdjust = -7;
  //   private static double m_launchRPMTopMultiplier = 1;

  // // public void set(Preset newPreset) {
  //   currentPreset = newPreset;
  // }

  // public Preset get() {
  //   return currentPreset;
  // }

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
    // println("Initializing PresetManager Dashboard");
    // SmartDashboard.putString("Presets/UI/SelectedCoralLevel", "");
    // SmartDashboard.putString("Presets/UI/SelectedCoralBranch", "");
    // SmartDashboard.putString("Presets/UI/SelectedAlgaeFace", "");
    // SmartDashboard.putString("Presets/UI/SelectedIntakeSide", "");

    // SmartDashboard.putStringArray(
    //     "Presets/UI/AllowedCoralLevels", UIConstants.allowedCoralLevels.toArray(new String[0]));
    // SmartDashboard.putStringArray(
    //     "Presets/UI/AllowedCoralBranches", UIConstants.allowedCoralBranches.toArray(new
    // String[0]));
    // SmartDashboard.putStringArray(
    //     "Presets/UI/allowedAlgaePositions",
    //     UIConstants.allowedAlgaePositions.toArray(new String[0]));
  }

  public void updateUIData() {
    // Get updated values from the UI
    CoralLevel coralLevel = ReefControlsIOServer.getSelectedCoralLevel();
    if (coralLevel != null) {
      uiSelectedCoralLevel2 = coralLevel;
      printf("Coral Level: %s", uiSelectedCoralLevel2.toString());
    }

    CoralBranch coralBranch = ReefControlsIOServer.getSelectedCoralBranch();
    if (coralBranch != null) {
      uiSelectedCoralBranch2 = coralBranch;
      printf("Coral Branch: %s", uiSelectedCoralBranch2.toString());
    }

    AlgaeFace2 algaeFace = ReefControlsIOServer.getSelectedAlgaeFace();
    if (algaeFace != null) {
      uiSelectedAlgaeFace2 = algaeFace;
      printf("Algae: %s", uiSelectedAlgaeFace2.toString());
    }

    PickupSide intakeSide = ReefControlsIOServer.getSelectedPickupSide();
    if (intakeSide != null) {
      uiSelectedIntakeSide2 = intakeSide;
      printf("Intake Side: %s", uiSelectedIntakeSide2.toString());
    }

    DTP dtp = ReefControlsIOServer.getSelectedDtp();
    if (dtp != null) {
      uiSelectedDtp2 = dtp;
      printf("DTP: %s", uiSelectedDtp2.toString());
    }
    // uiSelectedCoralLevel2 = ReefControlsIOServer.getSelectedCoralLevel();
    // uiSelectedIntakeSide2 = ReefControlsIOServer.getSelectedPickupSide();
    // uiSelectedCoralBranch2 = ReefControlsIOServer.getSelectedCoralBranch();
    // uiSelectedAlgaeFace2 = ReefControlsIOServer.getSelectedAlgaeFace();

    // Get Values from UI
    this.uiSelectedCoralLevel =
        SmartDashboard.getString("Presets/UI/SelectedCoralLevel", "UNKNOWN");
    // this.uiSelectedCoralBranch =
    //     SmartDashboard.getString("Presets/UI/SelectedCoralBranch", "UNKNOWN");

    this.uiSelectedAlgaeFace = SmartDashboard.getString("Presets/UI/SelectedAlgaeFace", "UNKNOWN");
    this.uiSelectedIntakeSide =
        SmartDashboard.getString("Presets/UI/SelectedIntakeSide", "UNKNOWN");

    this.uiSelectedCoralBranch = coralBranchSub.get();
    this.uiSelectedCoralLevel = coralLevelSub.get();
    this.uiSelectedAlgaeFace = algaeFaceSub.get();
    this.uiSelectedIntakeSide = intakeSideSub.get();

    // println("Getting Values " + uiSelectedCoralLevel);
    // println("               " + uiSelectedCoralBranch);
    // println("               " + uiSelectedAlgaeFace);
    // println("               " + uiSelectedIntakeSide);
  }

  private void updatePreset() {
    // New Selected Value for CORAL (Branch or Level)
    // Compute the new robot preset.
    // uiSelectedCoralLevel = "L1";
    // uiSelectedCoralBranch = "F";

    if ((!uiSelectedCoralLevel2.equals(this.uiCurrentCoralLevel2)
            && UIConstants.allowedCoralLevels2.contains(uiSelectedCoralLevel2))
        || (!uiSelectedCoralBranch2.equals(this.uiCurrentCoralBranch2)
            && UIConstants.allowedCoralBranches2.contains(uiSelectedCoralBranch2))) {

      printf(
          "Updating Presets - %s, %s",
          uiSelectedCoralBranch2.toString(), uiSelectedCoralLevel.toString());

      // New Selected Value for CORAL (Branch or Level)
      this.uiCurrentCoralBranch2 = uiSelectedCoralBranch2;
      this.uiCurrentCoralLevel2 = uiSelectedCoralLevel2;

      Pose2d branchPose =
          getBranchPoseFromTarget2(this.uiCurrentCoralBranch2, this.uiCurrentCoralLevel2);

      Pose2d robotPose =
          branchPose.plus(
              new Transform2d(
                  RobotConstants.robotCenterOffset.getX()
                      + Units.inchesToMeters(1.625), //  1.625 for Branch inset to face distance...
                  0,
                  branchPose.getRotation()));

      printf(
          "Transform: %f, %f, %f",
          RobotConstants.robotCenterOffset.getX(), 0.0, robotPose.getRotation().getDegrees());
      printf(
          "New Robot Pose - x: %3f, y: %3f, theta: %3f",
          robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());

      currentCoralPreset.setPose(robotPose);
      // TODO: Set Coral Level
    } else {
      // No new data or no valid data (to be handled)
    }

    // New Selected Value for ALGAE (Face)
    uiSelectedAlgaeFace = "IJ";

    if (!uiSelectedAlgaeFace2.equals(this.uiCurrentAlgaeFace2)
        && UIConstants.allowedAlgaeFaces2.contains(uiSelectedAlgaeFace2)) {

      printf("Updating Algae Presets - %s", uiSelectedAlgaeFace2.toString());

      this.uiCurrentAlgaeFace2 = uiSelectedAlgaeFace2;

      Pose2d facePose = getFacePoseFromTarget2(this.uiCurrentAlgaeFace2);
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

      currentAlgaePreset.setPose(goodRobotPose);
    } else {
      // No new data or no valid data (to be handled)
    }
  }

  private void updatePresetOLD() {
    // New Selected Value for CORAL (Branch or Level)
    // Compute the new robot preset.
    // uiSelectedCoralLevel = "L1";
    // uiSelectedCoralBranch = "F";

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
          "Transform: %f, %f, %f",
          RobotConstants.robotCenterOffset.getX(), 0.0, robotPose.getRotation().getDegrees());
      printf(
          "New Coral Pose - x: %3f, y: %3f, theta: %3f",
          robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());

      currentCoralPreset.setPose(robotPose);
      // TODO: Set Coral Level
    } else {
      // No new data or no valid data (to be handled)
    }

    // New Selected Value for ALGAE (Face)
    uiSelectedAlgaeFace = "IJ";

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
      //     facePose.transformBy(
      //         new Transform2d(RobotConstants.robotCenterOffset.getX(), 0,
      // facePose.getRotation()));

      // new Transform2d(
      //   RobotConstants.robotCenterOffset.getX(), 0, (facePose.getRotation()))

      //   ) = robotPose.rotateBy(new Rotation2d(Math.PI));

      printf(
          "New Algae Pose - x: %3f, y: %3f, theta: %3f",
          goodRobotPose.getX(), goodRobotPose.getY(), goodRobotPose.getRotation().getDegrees());

      currentAlgaePreset.setPose(goodRobotPose);
      // TODO: Set Algae Level
    } else {
      // No new data or no valid data (to be handled)
    }
  }

  private static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions.get(objective.branchId()).get(objective.reefLevel()).toPose2d();
  }

  private Pose2d getBranchPoseFromTarget2(CoralBranch branch, CoralLevel level) {
    ReefLevel branchHeight;
    int branchId = 0;

    switch (level) {
      case L1:
        branchHeight = ReefLevel.fromLevel(1);
        break;
      case L2:
        branchHeight = ReefLevel.fromLevel(2);
        break;
      case L3:
        branchHeight = ReefLevel.fromLevel(3);
        break;
      case L4:
      default:
        branchHeight = ReefLevel.fromLevel(4);
    }

    switch (branch) {
      case A:
        branchId = 1;
        break;
      case B:
        branchId = 0;
        break;
      case C:
        branchId = 11;
        break;
      case D:
        branchId = 10;
        break;
      case E:
        branchId = 9;
        break;
      case F:
        branchId = 8;
        break;
      case G:
        branchId = 7;
        break;
      case H:
        branchId = 6;
        break;
      case I:
        branchId = 5;
        break;
      case J:
        branchId = 4;
        break;
      case K:
        branchId = 3;
        break;
      case L:
        branchId = 2;
        break;
      default:
        branchId = 0;
        break;
    }
    return Reef.branchPositions.get(branchId).get(branchHeight).toPose2d();
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

  private Pose2d getFacePoseFromTarget2(AlgaeFace2 face) {
    int faceId = 0;
    boolean isLow = false;

    switch (face) {
      case AB:
        faceId = 0;
        isLow = false;
        break;
      case CD:
        faceId = 5;
        isLow = true;
        break;
      case EF:
        faceId = 4;
        isLow = false;
        break;
      case GH:
        faceId = 3;
        isLow = true;
        break;
      case IJ:
        faceId = 2;
        isLow = false;
        break;
      case KL:
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
