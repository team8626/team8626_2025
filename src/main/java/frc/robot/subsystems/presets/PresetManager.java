// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.RobotConstants.UIConstants.AlgaeFace2;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.RobotConstants.UIConstants.DTP;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants.CoralStation;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;

public class PresetManager extends CS_SubsystemBase {
  private UIConstants.CoralLevel uiSelectedCoralLevel2 = UIConstants.defaultCoralLevel;
  private UIConstants.PickupSide uiSelectedIntakeSide2 = UIConstants.defaultPickupSide;
  private UIConstants.CoralBranch uiSelectedCoralBranch2 = CoralBranch.NONE;
  private UIConstants.AlgaeFace2 uiSelectedAlgaeFace2 = AlgaeFace2.FLOOR;
  private UIConstants.DTP uiSelectedDtp2 = UIConstants.defaultDTP;

  private UIConstants.CoralLevel uiCurrentCoralLevel2 = UIConstants.defaultCoralLevel;
  private UIConstants.PickupSide uiCurrentIntakeSide2 = UIConstants.defaultPickupSide;
  private UIConstants.CoralBranch uiCurrentCoralBranch2 = CoralBranch.NONE;
  private UIConstants.AlgaeFace2 uiCurrentAlgaeFace2 = AlgaeFace2.FLOOR;
  private static UIConstants.DTP uiCurrentDtp2 = UIConstants.defaultDTP;

  // private final StringPublisher coralLevelPub =
  // NetworkTableInstance.getDefault().getStringTopic("SmartDashboard/Presets/UI/SelectedCoralLevel");
  // // NetworkTableInstance.getDefault()
  //     .getStringTopic("SmartDashboard/Presets/UI/SelectedCoralLevel");

  private static CoralPreset currentCoralPreset = new CoralPreset("autoPreset");
  private static AlgaePreset currentAlgaePreset = new AlgaePreset("autoPreset");

  StructPublisher<Pose2d> coralPresetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Subsystem/PresetManager/CoralPreset/Pose", Pose2d.struct)
          .publish();
  StructPublisher<Pose2d> algaePresetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Subsystem/PresetManager/AlgaePreset/Pose", Pose2d.struct)
          .publish();

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

    // Initialize the Presets
    currentCoralPreset.setLevel(uiSelectedCoralLevel2);
    currentAlgaePreset.setSubsystems(uiSelectedAlgaeFace2);
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

  public static AlgaePreset getAimAndShootPreset(Supplier<Pose2d> poseSupplier) {

    return getAimAndShootPreset(poseSupplier, () -> ElevatorConstants.minHeightInches);
  }

  public static AlgaePreset getAimAndShootPreset(
      Supplier<Pose2d> poseSupplier, DoubleSupplier elevatorHeightInches) {

    Pose2d robotPose = poseSupplier.get();
    AlgaePreset aimPreset = new AlgaePreset("AimPreset", 0, 0, 0);

    Pose3d targetPoseOurSide =
        AllianceFlipUtil.apply(
            new Pose3d(
                Units.inchesToMeters(325.68),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                new Rotation3d()));

    Pose3d targetPose2TheirSide =
        AllianceFlipUtil.apply(
            new Pose3d(
                Units.inchesToMeters(365.20),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                new Rotation3d()));
    String side;
    Pose3d targetPose;
    if (robotPose.getX() < targetPoseOurSide.getX()) {
      // We are on our side
      System.out.println("We are on our side");
      targetPose = targetPoseOurSide;
      side = "Our Side";
    } else if (robotPose.getX() > targetPose2TheirSide.getX()) {
      // We are on their side
      System.out.println("We are on their side");
      targetPose = targetPose2TheirSide;
      side = "Their Side";
    } else {
      // We are in the middle
      System.out.println("We are in the middle");
      targetPose = null;
      side = "Middle";
    }
    SmartDashboard.putString("Subsystem/Presets/AimPreset/SIDE", side);

    // // Compute Distance to Target (Only using X, assuming we are facing the NET)
    // if (targetPose != null) {

    //   // Compute Distance to Target (Only using X, assuming we are facing the NET)
    double x = Units.metersToFeet(Math.abs(robotPose.getX() - targetPoseOurSide.getX()));

    double wheelRadius = Units.metersToInches(AlgaeShooterConstants.wheelRadiusMeters);
    double launchVelocity = 24.1 + (x + 2) * (x + 2) / 70;

    double launchAngle = (x - 20) * (x - 20) / 11 + 55 + x / 5;
    double launchRpm = (launchVelocity * 12) / (2 * Math.PI * wheelRadius) * 60;

    double wristAngleDegrees = 180 - launchAngle;

    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AimPreset/RobotX (ft)", Units.metersToFeet(robotPose.getX()));
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AimPreset/RobotY (ft)", Units.metersToFeet(robotPose.getY()));
    SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Distance (ft)", x);
    SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Angle (deg)", launchAngle);
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AimPreset/Launch Velocity (ft.s-1)", launchVelocity);
    SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Speed (RPM)", launchRpm);
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AimPreset/WristAngle (deg)", wristAngleDegrees);
    SmartDashboard.putNumber(
        "Presets/AimPreset/ElevatorHeight (in)", elevatorHeightInches.getAsDouble());

    aimPreset.setElevatorHeightInches(elevatorHeightInches.getAsDouble());
    aimPreset.setWristAngleDegrees(wristAngleDegrees);
    aimPreset.setRPM(launchRpm);

    return aimPreset;
  }

  public void updateUIData() {
    // Get updated values from the UI
    CoralLevel coralLevel = Dashboard.getSelectedCoralLevel();
    if (coralLevel != null) {
      uiSelectedCoralLevel2 = coralLevel;
      // Will be processed in UdatePresets()
      printf("Coral Level: %s", uiSelectedCoralLevel2.toString());
    }

    CoralBranch coralBranch = Dashboard.getSelectedCoralBranch();
    if (coralBranch != null) {
      uiSelectedCoralBranch2 = coralBranch;
      // Will be processed in UdatePresets()
      printf("Coral Branch: %s", uiSelectedCoralBranch2.toString());
      Dashboard.setResetCoralBranch(false);
    }

    AlgaeFace2 algaeFace = Dashboard.getSelectedAlgaeFace();
    if (algaeFace != null) {
      uiSelectedAlgaeFace2 = algaeFace;
      // Will be processed in UdatePresets()
      printf("Algae: %s", uiSelectedAlgaeFace2.toString());
      Dashboard.setResetAlgaeFace(false);
    }

    PickupSide intakeSide = Dashboard.getSelectedPickupSide();
    if (intakeSide != null) {
      uiSelectedIntakeSide2 = intakeSide;
      uiCurrentIntakeSide2 = uiSelectedIntakeSide2;
      printf("Intake Side: %s", uiSelectedIntakeSide2.toString());
    }

    DTP dtp = Dashboard.getSelectedDtp();
    if (dtp != null) {
      uiSelectedDtp2 = dtp;
      printf("DTP: %s", uiSelectedDtp2.toString());
      uiCurrentDtp2 = uiSelectedDtp2;
    }
  }

  // Update the Presets based on the UI Data
  private void updatePreset() {
    // New Selected Value for CORAL (Branch or Level)
    if (!uiSelectedCoralLevel2.equals(this.uiCurrentCoralLevel2)
        && UIConstants.allowedCoralLevels2.contains(uiSelectedCoralLevel2)) {
      printf(
          "Updating Presets - %s, %s",
          uiSelectedCoralBranch2.toString(), uiSelectedCoralLevel2.toString());
      this.uiCurrentCoralLevel2 = uiSelectedCoralLevel2;

      currentCoralPreset.setLevel(this.uiCurrentCoralLevel2);
      Dashboard.setSelectedCoralLevel(this.uiSelectedCoralLevel2.getValue());
    }

    if (!uiSelectedCoralBranch2.equals(this.uiCurrentCoralBranch2)) {
      if (UIConstants.allowedCoralBranches.contains(uiSelectedCoralBranch2)) {
        printf(
            "Updating Presets - %s, %s",
            uiSelectedCoralBranch2.toString(), uiSelectedCoralLevel2.toString());
        this.uiCurrentCoralBranch2 = uiSelectedCoralBranch2;

        Pose2d branchPose =
            getBranchPoseFromTarget2(this.uiCurrentCoralBranch2, this.uiCurrentCoralLevel2);

        Pose2d robotPose =
            branchPose.plus(
                new Transform2d(
                    RobotConstants.robotCenterOffset.getX()
                        + Units.inchesToMeters(
                            1.625), //  1.625 for Branch inset to face distance...
                    0,
                    new Rotation2d())); /// branchPose.getRotation()));

        Pose2d robotPose2 =
            new Pose2d(
                robotPose.getX(),
                robotPose.getY(),
                new Rotation2d(robotPose.getRotation().getRadians()));

        printf(
            "Transform: %f, %f, %f",
            RobotConstants.robotCenterOffset.getX() + Units.inchesToMeters(1.625),
            0.0,
            robotPose.getRotation().getDegrees());
        printf(
            "Branch Pose - x: %3f, y: %3f, theta: %3f/n",
            branchPose.getX(), branchPose.getY(), branchPose.getRotation().getDegrees());
        printf(
            "New Robot Pose - x: %3f, y: %3f, theta: %3f",
            robotPose2.getX(), robotPose2.getY(), robotPose2.getRotation().getDegrees());

        currentCoralPreset.setPose(AllianceFlipUtil.apply(robotPose2));

      } else if (uiSelectedCoralBranch2.equals(CoralBranch.NONE)) {
        this.uiCurrentCoralBranch2 = uiSelectedCoralBranch2;
        currentCoralPreset.reset();
      }
      Dashboard.setSelectedCoralBranch(this.uiSelectedCoralBranch2.getValue());
    }

    // New Selected Value for ALGAE (Face)
    if (!uiSelectedAlgaeFace2.equals(this.uiCurrentAlgaeFace2)) {
      // New Selected Value for ALGAE is around the REEF
      if (UIConstants.allowedAlgaeFaces2.contains(uiSelectedAlgaeFace2)) {

        printf("Updating Algae Presets - %s", uiSelectedAlgaeFace2.toString());

        this.uiCurrentAlgaeFace2 = uiSelectedAlgaeFace2;

        Pose2d facePose = getFacePoseFromTarget2(this.uiCurrentAlgaeFace2);

        Pose2d robotPose =
            facePose.plus(
                new Transform2d(
                    RobotConstants.robotCenterOffset.getX()
                        + Units.inchesToMeters(0), //  1.625 for Branch inset to face distance...
                    0,
                    new Rotation2d())); /// branchPose.getRotation()));

        Pose2d robotPose2 =
            new Pose2d(
                robotPose.getX(),
                robotPose.getY(),
                new Rotation2d(robotPose.getRotation().getRadians() + Math.PI));

        printf(
            "Transform: %f, %f, %f\n",
            RobotConstants.robotCenterOffset.getX(), 0.0, robotPose.getRotation().getDegrees());
        printf(
            "Face Pose - x: %3f, y: %3f, theta: %3f\n",
            facePose.getX(), facePose.getY(), facePose.getRotation().getDegrees());
        printf(
            "New Algae Pose - x: %3f, y: %3f, theta: %3f\n",
            robotPose2.getX(), robotPose2.getY(), robotPose2.getRotation().getDegrees());

        currentAlgaePreset.setPose(AllianceFlipUtil.apply(robotPose2));
        currentAlgaePreset.setSubsystems(uiSelectedAlgaeFace2);
      }
      // New Selected Value for ALGAE is around the REEF ==> FLOOR
      else if (uiSelectedAlgaeFace2.equals(AlgaeFace2.FLOOR)) {
        printf("Updating Algae Presets - %s", uiSelectedAlgaeFace2.toString());

        this.uiCurrentAlgaeFace2 = uiSelectedAlgaeFace2;
        currentAlgaePreset.reset();
      }
      Dashboard.setSelectedAlgaeFace(this.uiCurrentAlgaeFace2.getValue());
    } else {
      // No new data or no valid data (to be handled)
    }
  }

  public static Pose2d getRobotPoseFromTarget(
      CoralBranch newBranch, CoralLevel newLevel, double newOffsetInches) {
    Pose2d retValue = new Pose2d();
    Pose2d branchPose = getBranchPoseFromTarget2(newBranch, newLevel);

    Pose2d robotPose =
        branchPose.plus(
            new Transform2d(
                RobotConstants.robotCenterOffset.getX()
                    + Units.inchesToMeters(1.625)
                    + Units.inchesToMeters(
                        newOffsetInches), //  1.625 for Branch inset to face distance...
                Units.inchesToMeters(newOffsetInches),
                new Rotation2d())); /// branchPose.getRotation()));

    Pose2d robotPose2 =
        new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            new Rotation2d(robotPose.getRotation().getRadians()));

    retValue = AllianceFlipUtil.apply(robotPose2);
    return retValue;
  }

  public static Pose2d getRobotPoseFromPickupSide(PickupSide newSide) {
    Pose2d retValue = new Pose2d();
    Pose2d sidepose = getPickupPoseFromPickupSide(newSide);

    Pose2d robotPose =
        sidepose.plus(
            new Transform2d(
                RobotConstants.robotCenterOffset.getX()
                    + Units.inchesToMeters(1.625), //  1.625 for Branch inset to face distance...
                0,
                new Rotation2d())); /// branchPose.getRotation()));

    Pose2d robotPose2 =
        new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            new Rotation2d(robotPose.getRotation().getRadians()));

    retValue = AllianceFlipUtil.apply(robotPose2);
    return retValue;
  }

  public static Pose2d getBranchPoseFromTarget2(CoralBranch branch, CoralLevel level) {
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
      case FLOOR:
      default:
        isLow = false;
        break;
    }

    printf(
        "New Face (%s - %s) - x: %3f, y: %3f, theta: %3f\n",
        face,
        isLow ? "Low" : "High",
        Reef.centerFaces[faceId].getX(),
        Reef.centerFaces[faceId].getY(),
        Reef.centerFaces[faceId].getRotation().getDegrees());

    return Reef.centerFaces[faceId];
  }

  private static Pose2d getPickupPoseFromPickupSide(PickupSide newSide) {
    Pose2d retValue = new Pose2d();

    switch (newSide) {
      case LEFT:
        retValue = CoralStation.leftCenterFace;
        break;
      case RIGHT:
      default:
        retValue = CoralStation.rightCenterFace;
        break;
    }
    return retValue;
  }

  public static boolean usingDtp() {
    return uiCurrentDtp2.getValue();
  }

  public static void resetCoralPreset() {
    Dashboard.setResetCoralBranch(true);
    currentCoralPreset.reset();
  }

  public static void resetAlgaePreset() {
    Dashboard.setResetAlgaeFace(true);
    currentAlgaePreset.reset();
  }

  // public static Command resetCoralPresetCmd() {
  //   Dashboard.resetCoralBranch();
  //   return new InstantCommand(() -> currentCoralPreset.reset());
  // }

  // public static Command resetAlgaePresetCmd() {
  //   Dashboard.resetAlgaeFace();
  //   return new InstantCommand(() -> currentAlgaePreset.reset());
  // }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/CoralPreset/RPMLeft", currentCoralPreset.getRPMLeft());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/CoralPreset/RPMRight", currentCoralPreset.getRPMRight());
    coralPresetPosePub.set(currentCoralPreset.getPose());

    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/ElevatorHeight",
        currentAlgaePreset.getElevatorHeightInches());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/RPM", currentAlgaePreset.getRPM());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/WristAngle",
        currentAlgaePreset.getWristAngleDegrees());
    algaePresetPosePub.set(currentAlgaePreset.getPose());
  }
}
