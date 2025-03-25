// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.presets;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.UIConstants;
import frc.robot.UIConstants.ALGAE_FACE;
import frc.robot.UIConstants.CORAL_BRANCH;
import frc.robot.UIConstants.CORAL_LEVEL;
import frc.robot.UIConstants.DTP;
import frc.robot.UIConstants.PICKUP_SIDE;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralStation;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;

public class PresetManager extends CS_SubsystemBase {
  private CORAL_LEVEL uiSelectedCoralLevel = UIConstants.defaultCoralLevel;
  private PICKUP_SIDE uiSelectedPickupSide = UIConstants.defaultPickupSide;
  private CORAL_BRANCH uiSelectedCoralBranch = CORAL_BRANCH.NONE;
  private ALGAE_FACE uiSelectedAlgaeFace = ALGAE_FACE.FLOOR;
  private DTP uiSelectedDtp2 = UIConstants.defaultDTP;
  private DTP uiSelectedDealgaefyDtp2 = UIConstants.defaultDealgaefyDTP;
  private DTP uiSelectedAlgaeShootDtp2 = UIConstants.defaultAlgaeShootDTP;

  private CORAL_LEVEL uiCurrentCoralLevel = UIConstants.defaultCoralLevel;
  private static PICKUP_SIDE uiCurrentPickupSide = UIConstants.defaultPickupSide;
  private CORAL_BRANCH uiCurrentCoralBranch = CORAL_BRANCH.NONE;
  private ALGAE_FACE uiCurrentAlgaeFace = ALGAE_FACE.FLOOR;
  private static DTP uiCurrentDtp2 = UIConstants.defaultDTP;
  private static DTP uiCurrentDealgaefyDtp2 = UIConstants.defaultDealgaefyDTP;
  private static DTP uiCurrentAlgaeShootDtp2 = UIConstants.defaultAlgaeShootDTP;

  private static CoralPreset currentCoralPreset = new CoralPreset("autoPreset");
  private static AlgaePreset currentAlgaePreset = new AlgaePreset("autoPreset");

  StructPublisher<Pose2d> coralPresetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Subsystem/PresetManager/CoralPreset/Pose", Pose2d.struct)
          .publish();
  StructPublisher<Pose2d> algaePresetDeaglaefyPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic(
              "SmartDashboard/Subsystem/PresetManager/AlgaePreset/DealgaefyPose", Pose2d.struct)
          .publish();
  StructPublisher<Pose2d> algaePresetShootPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic(
              "SmartDashboard/Subsystem/PresetManager/AlgaePreset/ShootPose", Pose2d.struct)
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
    currentCoralPreset.setLevel(uiSelectedCoralLevel);
    currentAlgaePreset.setSubsystems(uiSelectedAlgaeFace);
  }

  @Override
  public void CS_periodic() {
    updateUIData();
    updatePreset();
  }

  // Return the current presets
  public static Supplier<CoralPreset> getCoralPreset() {
    return () -> currentCoralPreset;
  }

  public static Supplier<AlgaePreset> getAlgaePreset() {
    return () -> currentAlgaePreset;
  }

  public static AlgaePreset getAimAndShootPreset(Supplier<Pose2d> poseSupplier) {
    return getAimAndShootPreset(poseSupplier, () -> ElevatorConstants.minHeight);
  }

  public static AlgaePreset getAimAndShootPreset(
      Supplier<Pose2d> poseSupplier, Supplier<Distance> elevatorHeight) {

    Pose2d robotPose = poseSupplier.get();
    AlgaePreset aimPreset = new AlgaePreset("AimPreset");

    // Pose2d targetPoseOurSide =
    //     AllianceFlipUtil.apply(
    //         new Pose2d(
    //             Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), new Rotation2d()));

    // Pose2d targetPose2TheirSide =
    //     AllianceFlipUtil.apply(
    //         new Pose2d(
    //             Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), new Rotation2d()));
    // String side;
    // Pose2d targetPose;
    // if (robotPose.getX() < targetPoseOurSide.getX()) {
    //   // We are on our side
    //   targetPose = targetPoseOurSide;
    //   side = "Our Side";
    // } else if (robotPose.getX() > targetPose2TheirSide.getX()) {
    //   // We are on their side
    //   targetPose = targetPose2TheirSide;
    //   side = "Their Side";
    // } else {
    //   // We are in the middle
    //   targetPose = new Pose2d();
    //   side = "Middle";
    // }

    // // Compute Distance to Target (Only using X, assuming we are facing the NET)
    // double x = 0, launchVelocity = 0, launchAngle = 0, launchRpm = 0;
    // Angle wristAngle = Degrees.of(0);

    // Distance wheelRadius = AlgaeShooterConstants.wheelRadius;

    // if (targetPose != new Pose2d()) {

    //   //   // Compute Distance to Target (Only using X, assuming we are facing the NET)
    //   x = Units.metersToFeet(Math.abs(robotPose.getX() - targetPose.getX()));

    //   launchVelocity = 24.1 + (x + 2) * (x + 2) / 70;

    //   launchAngle = (x - 20) * (x - 20) / 11 + 55 + x / 5;
    //   launchRpm = (launchVelocity * 12) / (wheelRadius.times(2* Math.PI * 60);

    //   wristAngleDegrees = 180 - launchAngle;
    // }

    // SmartDashboard.putString("Subsystem/Presets/AimPreset/Side", side);
    // SmartDashboard.putNumber(
    //     "Subsystem/PresetManager/AimPreset/RobotX (ft)", Units.metersToFeet(robotPose.getX()));
    // SmartDashboard.putNumber(
    //     "Subsystem/PresetManager/AimPreset/RobotY (ft)", Units.metersToFeet(robotPose.getY()));
    // SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Distance (ft)", x);
    // SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Angle (deg)",
    // launchAngle);
    // SmartDashboard.putNumber(
    //     "Subsystem/PresetManager/AimPreset/Launch Velocity (ft.s-1)", launchVelocity);
    // SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/Launch Speed (RPM)", launchRpm);
    // SmartDashboard.putNumber(
    //     "Subsystem/PresetManager/AimPreset/WristAngle (deg)", wristAngleDegrees);
    // SmartDashboard.putNumber(
    //     "Presets/AimPreset/ElevatorHeight (in)", elevatorHeightInches.get().in(Inches));

    // aimPreset.setElevatorHeight(elevatorHeight.get().in(Inches));
    // aimPreset.setWristAngle(wristAngle);
    // aimPreset.setRPM(launchRpm);

    return aimPreset;
  }

  /**
   * Get the preset for the Barge Shoot depending on the side of the field
   *
   * @param robotPoseSupplier Current Pose of the Robot
   * @return AlgaePreset
   */
  public static AlgaePreset getBargeShootPreset(Supplier<Pose2d> robotPoseSupplier) {
    AlgaePreset retVal = Presets.ALGAE_SHOOTBARGE_OURSIDE;
    Pose2d robotPose = robotPoseSupplier.get();

    if (robotPose.getX() > (FieldConstants.fieldLength / 2)) {
      // We are on their side
      retVal = Presets.ALGAE_SHOOTBARGE_THEIRSIDE;
    }

    retVal.setPose(AllianceFlipUtil.apply(retVal.getShootingPose()));
    return retVal;
  }

  public void updateUIData() {
    // Get updated values from the UI
    CORAL_LEVEL coralLevel = Dashboard.getSelectedCoralLevel();
    if (coralLevel != null) {
      uiSelectedCoralLevel = coralLevel;
      // Will be processed in UdatePresets()
      printf("Coral Level: %s", uiSelectedCoralLevel.toString());
    }

    CORAL_BRANCH coralBranch = Dashboard.getSelectedCoralBranch();
    if (coralBranch != null) {
      uiSelectedCoralBranch = coralBranch;
      // Will be processed in UdatePresets()
      printf("Coral Branch: %s", uiSelectedCoralBranch.toString());
      Dashboard.setResetCoralBranch(false);
    }

    ALGAE_FACE algaeFace = Dashboard.getSelectedAlgaeFace();
    if (algaeFace != null) {
      uiSelectedAlgaeFace = algaeFace;
      // Will be processed in UdatePresets()
      printf("Algae: %s", uiSelectedAlgaeFace.toString());
      Dashboard.setResetAlgaeFace(false);
    }

    PICKUP_SIDE pickupSide = Dashboard.getSelectedPickupSide();
    if (pickupSide != null) {
      uiSelectedPickupSide = pickupSide;
      uiCurrentPickupSide = uiSelectedPickupSide;
      printf("Pickup Side: %s", uiSelectedPickupSide.toString());
    }

    DTP dtp = Dashboard.getSelectedDtp();
    if (dtp != null) {
      uiSelectedDtp2 = dtp;
      printf("Coral Shoot DTP: %s", uiSelectedDtp2.toString());
      uiCurrentDtp2 = uiSelectedDtp2;
    }

    DTP dealgaefyDtp = Dashboard.getSelectedDealgaefyDtp();
    if (dealgaefyDtp != null) {
      uiSelectedDealgaefyDtp2 = dealgaefyDtp;
      printf("Dealgaefy DTP: %s", uiSelectedDealgaefyDtp2.toString());
      uiCurrentDealgaefyDtp2 = uiSelectedDealgaefyDtp2;
    }
    DTP algaeShootdtp = Dashboard.getSelectedAlgaeShootDtp();
    if (algaeShootdtp != null) {
      uiSelectedAlgaeShootDtp2 = algaeShootdtp;
      printf("Algae Shoot DTP: %s", uiSelectedAlgaeShootDtp2.toString());
      uiCurrentAlgaeShootDtp2 = uiSelectedAlgaeShootDtp2;
    }
  }

  // Update the Presets based on the UI Data
  private void updatePreset() {
    // New Selected Value for CORAL Level)
    if (!uiSelectedCoralLevel.equals(this.uiCurrentCoralLevel)
        && UIConstants.ALLOWED_CORAL_LEVELS.contains(uiSelectedCoralLevel)) {
      printf(
          "Updating Presets - %s, %s",
          uiSelectedCoralBranch.toString(), uiSelectedCoralLevel.toString());
      this.uiCurrentCoralLevel = uiSelectedCoralLevel;

      currentCoralPreset.setLevel(this.uiCurrentCoralLevel);
      Dashboard.setSelectedCoralLevel(this.uiSelectedCoralLevel.getValue());
    }

    // New Selected Value for CORAL Branch
    if (!uiSelectedCoralBranch.equals(this.uiCurrentCoralBranch)) {
      if (UIConstants.ALLOWED_CORAL_BRANCHES.contains(uiSelectedCoralBranch)) {
        printf(
            "Updating Presets - %s, %s",
            uiSelectedCoralBranch.toString(), uiSelectedCoralLevel.toString());
        this.uiCurrentCoralBranch = uiSelectedCoralBranch;

        Pose2d branchPose =
            getBranchPoseFromTarget(this.uiCurrentCoralBranch, this.uiCurrentCoralLevel);

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

      } else if (uiSelectedCoralBranch.equals(CORAL_BRANCH.NONE)) {
        this.uiCurrentCoralBranch = uiSelectedCoralBranch;
        currentCoralPreset.reset();
      }
      Dashboard.setSelectedCoralBranch(this.uiSelectedCoralBranch.getValue());
    }

    // New Selected Value for ALGAE (Face)
    if (!uiSelectedAlgaeFace.equals(this.uiCurrentAlgaeFace)) {
      // New Selected Value for ALGAE is around the REEF
      if (UIConstants.ALLOWED_ALGAE_FACES.contains(uiSelectedAlgaeFace)) {

        printf("Updating Algae Presets - %s", uiSelectedAlgaeFace.toString());

        this.uiCurrentAlgaeFace = uiSelectedAlgaeFace;

        Pose2d facePose = getFacePoseFromTarget(this.uiCurrentAlgaeFace);

        Pose2d robotPose =
            facePose.plus(
                new Transform2d(
                    RobotConstants.robotCenterOffset.getX() + Units.inchesToMeters(0),
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
        currentAlgaePreset.setSubsystems(uiSelectedAlgaeFace);
      }
      // New Selected Value for ALGAE is around the REEF ==> FLOOR
      else if (uiSelectedAlgaeFace.equals(ALGAE_FACE.FLOOR)) {
        printf("Updating Algae Presets - %s", uiSelectedAlgaeFace.toString());

        this.uiCurrentAlgaeFace = uiSelectedAlgaeFace;
        currentAlgaePreset.reset();
      }
      Dashboard.setSelectedAlgaeFace(this.uiCurrentAlgaeFace.getValue());
    } else {
      // No new data or no valid data (to be handled)
    }
  }

  /**
   * Get the Robot Pose from the Target
   *
   * @param newBranch Coral Branch (A-L)
   * @param newLevel Coral Level (L1-L4)
   * @return Pose2d (new Pose2d() is the Branch is NONE or Invalid)
   */
  public static Pose2d getRobotPoseFromTarget(CORAL_BRANCH newBranch, CORAL_LEVEL newLevel) {
    return getRobotPoseFromTarget(newBranch, newLevel, 0);
  }

  /**
   * Get the Robot Pose from the Target with an Offset
   *
   * @param newBranch Coral Branch (A-L)
   * @param newLevel Coral Level (L1-L4)
   * @param newOffsetInches Offset in Inches (from the Target)
   * @return Pose2d (new Pose2d() is the Branch is NONE or Invalid)
   */
  public static Pose2d getRobotPoseFromTarget(
      CORAL_BRANCH newBranch, CORAL_LEVEL newLevel, double newOffsetInches) {
    Pose2d retValue = new Pose2d();
    Pose2d branchPose = getBranchPoseFromTarget(newBranch, newLevel);

    if (!branchPose.equals(new Pose2d())) {
      Pose2d robotPose =
          branchPose.plus(
              new Transform2d(
                  RobotConstants.robotCenterOffset.getX()
                      + Units.inchesToMeters(1.625)
                      + Units.inchesToMeters(
                          newOffsetInches), //  1.625 for Branch inset to face distance...
                  0,
                  new Rotation2d())); /// branchPose.getRotation()));

      Pose2d robotPose2 =
          new Pose2d(
              robotPose.getX(),
              robotPose.getY(),
              new Rotation2d(robotPose.getRotation().getRadians()));

      retValue = AllianceFlipUtil.apply(robotPose2);
    }

    return retValue;
  }

  /**
   * Get the Robot Pose from the Target with an Offset
   *
   * @param newFace Algae Face (AB - KL)
   * @param newOffsetInches Offset in Inches (from the Target)
   * @return Pose2d (new Pose2d() is the Face is NONE or Invalid)
   */
  public static Pose2d getRobotPoseFromAlgae(ALGAE_FACE newFace, double newOffsetInches) {
    Pose2d retValue = new Pose2d();
    Pose2d facePose = getFacePoseFromTarget(newFace);

    if (!facePose.equals(new Pose2d())) {
      Pose2d robotPose =
          facePose.plus(
              new Transform2d(
                  RobotConstants.robotCenterOffset.getX() + Units.inchesToMeters(0),
                  0,
                  new Rotation2d())); /// branchPose.getRotation()));

      Pose2d robotPose2 =
          new Pose2d(
              robotPose.getX(),
              robotPose.getY(),
              new Rotation2d(robotPose.getRotation().getRadians() + Math.PI));

      retValue = AllianceFlipUtil.apply(robotPose2);
    }

    return retValue;
  }

  /**
   * Get the Robot Pose from the Pickup Side
   *
   * @param newSide Pickup Side (LEFT/RIGHT)
   * @return Pose2d (new Pose2d() is the Side is NONE)
   */
  public static Pose2d getRobotPoseFromPickupSide(Supplier<PICKUP_SIDE> newSide) {
    Pose2d retValue = new Pose2d();

    Pose2d sidepose = getPickupPoseFromPickupSide(newSide.get());
    if (!sidepose.equals(new Pose2d())) {
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
    }
    return retValue;
  }

  /**
   * Get the Branch Pose from the Target
   *
   * @param branch Coral Branch (A-L)
   * @param level Coral Level (L1-L4)
   * @return Pose2d (new Pose2d() is the Branch is NONE or Invalid)
   */
  public static Pose2d getBranchPoseFromTarget(CORAL_BRANCH branch, CORAL_LEVEL level) {
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
      case NONE:
      default:
        branchId = -1;
        break;
    }
    return branchId == -1
        ? new Pose2d()
        : Reef.branchPositions.get(branchId).get(branchHeight).toPose2d();
  }

  /**
   * Get the Face Pose from the Target
   *
   * @param face ALGAE_FACE (AB, CD, EF, GH, IJ, KL, FLOOR)
   * @return Pose2d (new Pose2d() is the Face is FLOOR or Invalid)
   */
  private static Pose2d getFacePoseFromTarget(ALGAE_FACE face) {
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

    System.out.printf(
        "New Face (%s - %s) - x: %3f, y: %3f, theta: %3f\n",
        face,
        isLow ? "Low" : "High",
        Reef.centerFaces[faceId].getX(),
        Reef.centerFaces[faceId].getY(),
        Reef.centerFaces[faceId].getRotation().getDegrees());

    return Reef.centerFaces[faceId];
  }

  public static Pose2d getPickupPoseFromPickupSide(PICKUP_SIDE newSide) {
    Pose2d retValue = new Pose2d();

    switch (newSide) {
      case LEFT:
        retValue = CoralStation.leftCenterFace;
        break;
      case RIGHT:
        retValue = CoralStation.rightCenterFace;
        break;
      case NONE:
      default:
        retValue = new Pose2d();
        break;
    }
    return retValue;
  }

  public static BooleanSupplier usingCoralShootDtp() {
    return () -> uiCurrentDtp2.getValue();
  }

  public static BooleanSupplier usingDealgaefyDtp() {
    return () -> uiCurrentDealgaefyDtp2.getValue();
  }

  public static BooleanSupplier usingAlgaeShootDtp() {
    return () -> uiCurrentAlgaeShootDtp2.getValue();
  }

  public static Supplier<PICKUP_SIDE> getPickupSide() {
    return () -> uiCurrentPickupSide;
  }

  public static void resetCoralPreset_OLD() {
    Dashboard.setResetCoralBranch(true);
    currentCoralPreset.reset();
  }

  public static void resetAlgaePreset_OLD() {
    Dashboard.setResetAlgaeFace(true);
    currentAlgaePreset.reset();
  }

  @Override
  public void updateDashboard() {
    coralPresetPosePub.set(currentCoralPreset.getPose());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/CoralPreset/RPMLeft", currentCoralPreset.getRPMLeft());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/CoralPreset/RPMRight", currentCoralPreset.getRPMRight());

    algaePresetDeaglaefyPosePub.set(currentAlgaePreset.getDealgaefyPose());
    algaePresetShootPosePub.set(currentAlgaePreset.getShootingPose());
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/ElevatorHeight",
        currentAlgaePreset.getElevatorHeight().in(Inches));
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/RPM", currentAlgaePreset.getRPM().in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/PresetManager/AlgaePreset/WristAngle",
        currentAlgaePreset.getWristAngle().in(Degrees));
  }
}
