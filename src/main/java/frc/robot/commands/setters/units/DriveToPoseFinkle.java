// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPoseFinkle extends CS_Command {

  private final SwerveSubsystem m_drive;

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

  private static final double defaultPositionTolerance = 0.01; // meters
  private static final double defaultPositionVelocityTolerance = 0.01; // meters
  private static final double defaultRotationToleranceRadians = Units.degreesToRadians(1.0);
  private static final double defaultRotationVelocityTolerance = Units.degreesToRadians(1.0);

  private double defaultDriveP_X = 0.76;
  private double defaultDriveP_Y = 0.76;
  private double defaultRotP = 5;

  private StructPublisher<Pose2d> targetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/DesiredPose", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> currentPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/CurrentPose", Pose2d.struct)
          .publish();

  private boolean hasValidPose = false;

  private double positionTolerance = defaultPositionTolerance;
  private double rotationToleranceRadians = defaultRotationToleranceRadians;

  public DriveToPoseFinkle(Supplier<Pose2d> desiredPoseSupplier) {
    this(
        desiredPoseSupplier, () -> defaultPositionTolerance, () -> defaultRotationToleranceRadians);
  }

  /**
   * Creates a new DriveToPoseFinkle command.
   *
   * @param desiredPoseSupplier
   * @param posToleranceSupplier in inches
   * @param rotToleranceDegreeSupplier in Degrees
   */
  public DriveToPoseFinkle(
      Supplier<Pose2d> desiredPoseSupplier,
      DoubleSupplier posToleranceSupplier,
      DoubleSupplier rotToleranceDegreeSupplier) {
    m_drive = RobotContainer.drivebase;

    positionTolerance = posToleranceSupplier.getAsDouble();
    rotationToleranceRadians = rotToleranceDegreeSupplier.getAsDouble();

    m_desiredPoseSupplier = desiredPoseSupplier;

    setName("DRIVETOPOSEFINKLE");

    targetPosePub.set(new Pose2d());
    currentPosePub.set(new Pose2d());

    this.initDashboard();
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_drive.resetOdometry();

    hasValidPose = false;
    if (m_desiredPoseSupplier != null) {
      if (m_desiredPoseSupplier.get() != null) {
        hasValidPose = true;
      }
    }

    if (hasValidPose == true) {
      Commodore.setCommodoreState(CommodoreState.DRIVE_FINKLE);
      m_pose = m_drive.getPose();

      targetPosePub.set(m_desiredPoseSupplier.get());
      currentPosePub.set(m_pose);

      m_xDesiredPos = m_desiredPoseSupplier.get().getX();
      m_yDesiredPos = m_desiredPoseSupplier.get().getY();
      m_desiredRotRadians = m_desiredPoseSupplier.get().getRotation().getRadians();

      Commands.print("(" + m_xDesiredPos + "  " + m_yDesiredPos + ")").schedule();

      double drivePValueX =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/P", defaultDriveP_X);
      double driveIValueX =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/I", 0);
      double driveDValueX =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/D", 0);

      double drivePValueY =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/P", defaultDriveP_Y);
      double driveIValueY =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/I", 0);
      double driveDValueY =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/D", 0);

      double rotPValue =
          SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/P", defaultRotP);
      double rotIValue = SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/I", 0);
      double rotDValue = SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/Rotation/D", 0);

      // Max Velocity and Acceleration
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

      // Set the constraints for the PID controllers
      m_xPID.setConstraints(
          new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));
      m_yPID.setConstraints(
          new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));
      m_rotPID.setConstraints(
          new TrapezoidProfile.Constraints(newRotMaxVelocity, newRotMaxAcceleration));

      // Set the PID values
      m_xPID.setPID(drivePValueX, driveIValueX, driveDValueX);
      m_yPID.setPID(drivePValueY, driveIValueY, driveDValueY);
      m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

      // Set the tolerances
      m_xPID.setTolerance(positionTolerance, defaultPositionVelocityTolerance);
      m_yPID.setTolerance(positionTolerance, defaultPositionVelocityTolerance);
      m_rotPID.setTolerance(rotationToleranceRadians, defaultRotationVelocityTolerance);

      // Reset the PID controllers
      m_xPID.reset(m_pose.getX());
      m_yPID.reset(m_pose.getY());
      m_rotPID.reset(m_pose.getRotation().getRadians());
      m_rotPID.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public void execute() {
    if (hasValidPose) {
      m_pose = m_drive.getPose();

      targetPosePub.set(m_desiredPoseSupplier.get());
      currentPosePub.set(m_pose);

      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/CalculateX", m_xPID.calculate(m_pose.getX(), m_xDesiredPos));
      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/CalculateY", m_yPID.calculate(m_pose.getY(), m_yDesiredPos));
      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/CalculateTheta",
          m_rotPID.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians));

      m_drive
          .driveCommand(
              () -> m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
              () -> m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
              () -> m_rotPID.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians),
              () -> true)
          .execute();

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
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Commands/DriveToPoseFinkle/XAtSetpoint", m_xPID.atSetpoint());
    SmartDashboard.putBoolean("Commands/DriveToPoseFinkle/YAtSetpoint", m_yPID.atSetpoint());
    SmartDashboard.putBoolean("Commands/DriveToPoseFinkle/RotAtSetpoint", m_rotPID.atSetpoint());

    return hasValidPose && (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint());
  }

  void initDashboard() {
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionX/P",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/P", defaultDriveP_X));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionX/I",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionX/D",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionX/D", 0.0));

    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionY/P",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/P", defaultDriveP_Y));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionY/I",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/Gains/PositionY/D",
        SmartDashboard.getNumber("Commands/DriveToPoseFinkle/Gains/PositionY/D", 0.0));

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
        "Commands/DriveToPoseFinkle/RotationTolerance(deg)",
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
  }
}
