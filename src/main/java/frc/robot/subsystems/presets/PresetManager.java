// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.subsystems.CS_SubsystemBase;
import java.util.Optional;

public class PresetManager extends CS_SubsystemBase {
  private String uiSelectedCORALLevel = "L1";
  private String uiSelectedIntakeSide = "UNKNOWN";
  private String uiSelectedREEFBranch = "UNKNOWN";
  private String uiSelectedAlgaePosition = "UNKNOWN";
  private String uiCurrentCORALLevel = "L4";
  private String uiCurrentIntakeSide = "UNKNOWN";
  private String uiCurrentREEFBranch = "";
  private String uiCurrentAlgaePosition = "UNKNOWN";
  // private CoralPreset autoCoralPreset = new CoralPreset( "autoPreset", new Pose2d(), 0.0, false);

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

  private StructPublisher<Pose2d> robotPostPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Presets/Pose2d", Pose2d.struct)
          .publish();

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
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    // if (this.currentPreset != null) {
    //   robotPostPublisher.set(currentPreset.robotPose());
    //   SmartDashboard.putString("Presets/Preset", currentPreset.Name());
    //   SmartDashboard.putNumber("Presets/Elevator Height", currentPreset.elevatorHeightInches());
    //   SmartDashboard.putNumber("Presets/Wrist Angle", currentPreset.WristAngleDegrees());
    //   SmartDashboard.putNumber("Presets/Algae RPM", currentPreset.AlgaeRPM());
    //   SmartDashboard.putNumber("Presets/Coral RPM", currentPreset.CoralRPM());
    //   SmartDashboard.putNumber("Presets/Robot Pose X", currentPreset.robotPose().getX());
    //   SmartDashboard.putNumber("Presets/Robot Pose Y", currentPreset.robotPose().getY());
    // }
  }

  public void updateUIData() {
    // TODO Write this Function for the PresetManager
    // SmartDashboard.putStringArray("Presets/UI/AllowedCORALLevels",
    // UIConstants.allowedCORALLevels);

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
    uiSelectedCORALLevel = SmartDashboard.getString("Presets/UI/SelectedCORALLevel", "UNKNOWN");
    uiSelectedREEFBranch = SmartDashboard.getString("Presets/UI/SelectedREEFBranch", "UNKNOWN");
    uiSelectedIntakeSide = SmartDashboard.getString("Presets/UI/SelectedIntakeSide", "UNKNOWN");
    uiSelectedAlgaePosition =
        SmartDashboard.getString("Presets/UI/SelectedAlgaePosition", "UNKNOWN");
  }

  private void updatePreset() {
    if ((!uiSelectedCORALLevel.equals(uiCurrentCORALLevel)
            && UIConstants.allowedCORALLevels.contains(uiSelectedCORALLevel))
        || (!uiSelectedREEFBranch.equals(uiCurrentREEFBranch)
            && UIConstants.allowedREEFBranches.contains(uiSelectedREEFBranch))) {
      // Insert a placeholder for creating a new preset
      // autoCoralPreset.setRPM(1275.0);
      // autoCoralPreset.setPose(new Pose2d());
    }
  }

  private Pose2d getRobotPoseFromBranch(String branch) {
    double branchOffsetX = Units.inchesToMeters(14.75 / 2) * Math.cos(30);
    double branchOffsetY = Units.inchesToMeters(14.75 / 2) * Math.sin(30);
    Pose2d branchPose;

    switch (branch) {
      case "BranchA":
        branchPose =
            FieldConstants.Reef.centerFaces[0].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchB":
        branchPose =
            FieldConstants.Reef.centerFaces[0].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchC":
        branchPose =
            FieldConstants.Reef.centerFaces[1].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchD":
        branchPose =
            FieldConstants.Reef.centerFaces[1].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchE":
        branchPose =
            FieldConstants.Reef.centerFaces[2].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchF":
        branchPose =
            FieldConstants.Reef.centerFaces[2].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchG":
        branchPose =
            FieldConstants.Reef.centerFaces[3].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchH":
        branchPose =
            FieldConstants.Reef.centerFaces[3].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchI":
        branchPose =
            FieldConstants.Reef.centerFaces[4].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchJ":
        branchPose =
            FieldConstants.Reef.centerFaces[4].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchK":
        branchPose =
            FieldConstants.Reef.centerFaces[5].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      case "BranchL":
        branchPose =
            FieldConstants.Reef.centerFaces[5].plus(
                new Transform2d(
                    new Translation2d(branchOffsetX, branchOffsetY), Rotation2d.fromDegrees(30)));
        break;
      default:
        throw new IllegalArgumentException("Unknown branch: " + branch);
    }

    //  Y Deepest Point -  Y Shallowest Point = dY
    // X Deep Point - X Shallow Point = dX

    Pose2d retVal = new Pose2d();

    return retVal;
  }
}
