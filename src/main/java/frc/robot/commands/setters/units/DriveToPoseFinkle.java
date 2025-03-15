// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPoseFinkle extends CS_Command {

  private final CS_DriveSubsystem m_drive;

  private final ProfiledPIDController m_xPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_yPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_rotPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Supplier<Pose2d> m_desiredPoseSupplier;
  private Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;
  private double m_desiredRotRadians;

  private double defaultPositionMaxVelocity = 6;
  private double defaultPositionMaxAcceleration = 8;

  private double defaultRotationMaxVelocity = Units.degreesToRadians(720);
  private double defaultRotationMaxAcceleration = Units.degreesToRadians(360);

  private final double defaultPositionTolerance = 0.01; // meters
  private final double defaultPositionVelocityTolerance = 0.01; // meters
  private final double defaultRotationToleranceRadians = Units.degreesToRadians(1.0);
  private final double defaultRotationVelocityTolerance = Units.degreesToRadians(1.0);

  private double defaultDriveP = 0.57;
  private double defaultRotP = Units.degreesToRadians(1);

  private StructPublisher<Pose2d> targetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/DesiredPose", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> currentPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/CurrentPose", Pose2d.struct)
          .publish();

  private boolean m_finish;
  private boolean hasValidPose = false;

  private double positionTolerance = defaultPositionTolerance;
  private double rotationToleranceRadians = defaultPositionVelocityTolerance;

  public DriveToPoseFinkle(
      Supplier<Pose2d> desiredPoseSupplier,
      DoubleSupplier posToleranceSupplier,
      DoubleSupplier rotToleranceDegreeSupplier) {
    this(desiredPoseSupplier, true);

    positionTolerance = posToleranceSupplier.getAsDouble();
    rotationToleranceRadians = Units.degreesToRadians(rotToleranceDegreeSupplier.getAsDouble());
  }

  public DriveToPoseFinkle(Supplier<Pose2d> desiredPoseSupplier, boolean finish) {
    m_drive = RobotContainer.drivebase;

    m_desiredPoseSupplier = desiredPoseSupplier;
    m_finish = finish;

    addRequirements(m_drive);
    setName("DRIVETOPOSEFINKLE");

    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Position/P",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/P", defaultDriveP));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Position/I",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Position/D",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/D", 0.0));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Rotation/P",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/P", defaultRotP));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Rotation/I",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/Rotation/D",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/D", 0.0));

    SmartDashboard.putNumber("Commands/DriveToPoseFinkle/PositionTolerance(m)", positionTolerance);
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationTolerance(m)",
        Units.radiansToDegrees(rotationToleranceRadians));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/PositionVelocityTolerance(m.s-2)",
        defaultPositionVelocityTolerance);
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/DriveVelocityConstraint", defaultPositionMaxVelocity);
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/DriveAccelerationConstraint", defaultPositionMaxAcceleration);
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationVelocityConstraint", defaultRotationMaxVelocity);
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationAccelerationConstraint",
        defaultRotationMaxAcceleration);

    targetPosePub.set(new Pose2d());
    currentPosePub.set(new Pose2d());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    hasValidPose = false;
    if (m_desiredPoseSupplier != null) {
      if (m_desiredPoseSupplier.get() != null) {
        hasValidPose = true;
      }
    }

    if (hasValidPose == true) {
      Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);
      m_pose = m_drive.getPose2d();

      targetPosePub.set(m_desiredPoseSupplier.get());
      currentPosePub.set(m_pose);

      m_xDesiredPos = m_desiredPoseSupplier.get().getX();
      m_yDesiredPos = m_desiredPoseSupplier.get().getY();
      m_desiredRotRadians = m_desiredPoseSupplier.get().getRotation().getRadians();

      Commands.print("(" + m_xDesiredPos + "  " + m_yDesiredPos + ")").schedule();

      double drivePValue =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/P", defaultDriveP);
      double driveIValue =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/I", 0);
      double driveDValue =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/D", 0);

      double rotPValue =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/P", defaultRotP);
      double rotIValue = SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/I", 0);
      double rotDValue = SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Position/D", 0);

      double newDriveMaxVelocity =
          SmartDashboard.getNumber(
              "Commands/DriveToPoseFinkle/DriveVelocityConstraint", defaultPositionMaxVelocity);
      double newDriveMaxAcceleration =
          SmartDashboard.getNumber(
              "Commands/DriveToPoseFinkle/DriveAccelerationConstraint",
              defaultPositionMaxAcceleration);

      double newRotMaxVelocity =
          SmartDashboard.getNumber(
              "Commands/DriveToPoseFinkle/RotationVelocityConstraint", defaultRotationMaxVelocity);
      double newRotMaxAcceleration =
          SmartDashboard.getNumber(
              "Commands/DriveToPoseFinkle/RotationAccelerationConstraint",
              defaultRotationMaxAcceleration);

      m_xPID.setConstraints(
          new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));
      m_yPID.setConstraints(
          new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));
      m_rotPID.setConstraints(
          new TrapezoidProfile.Constraints(newRotMaxVelocity, newRotMaxAcceleration));

      m_xPID.setPID(drivePValue, driveIValue, driveDValue);
      m_yPID.setPID(drivePValue, driveIValue, driveDValue);
      m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

      m_xPID.setTolerance(positionTolerance, defaultPositionVelocityTolerance);
      m_yPID.setTolerance(positionTolerance, defaultPositionVelocityTolerance);
      m_rotPID.setTolerance(rotationToleranceRadians, defaultRotationVelocityTolerance);

      m_xPID.reset(m_pose.getX());
      m_yPID.reset(m_pose.getY());
      m_rotPID.reset(m_pose.getRotation().getRadians());
    }
  }

  @Override
  public void execute() {
    if (hasValidPose) {

      m_pose = m_drive.getPose2d();

      targetPosePub.set(m_desiredPoseSupplier.get());
      currentPosePub.set(m_pose);

      m_drive.drive(
          new ChassisSpeeds(
              m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
              m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
              m_rotPID.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians)));

      SmartDashboard.putNumber("Commands/DriveToPoseFinkle/ErrorX", m_xPID.getPositionError());
      SmartDashboard.putNumber("Commands/DriveToPoseFinkle/ErrorY", m_yPID.getPositionError());
      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/ErrorTheta",
          Units.radiansToDegrees(m_rotPID.getPositionError()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    Commodore.setCommodoreState(CommodoreState.IDLE);

    // Reset Values
    positionTolerance = defaultPositionTolerance;
    rotationToleranceRadians = defaultRotationToleranceRadians;
  }

  @Override
  public boolean isFinished() {
    return hasValidPose
        && (m_finish && m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint());
  }
}
