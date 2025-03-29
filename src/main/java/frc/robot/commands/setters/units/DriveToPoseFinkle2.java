// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.Supplier;

public class DriveToPoseFinkle2 extends CS_Command {

  private final SwerveSubsystem m_drive;

  private final ProfiledPIDController xPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController yPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController rotPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Supplier<Pose2d> m_desiredPoseSupplier;
  private Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;
  private double m_desiredRotRadians;

  private LinearVelocity defaultPositionMaxVelocity = MetersPerSecond.of(6);
  private LinearAcceleration defaultPositionMaxAcceleration = MetersPerSecondPerSecond.of(8);

  private AngularVelocity defaultRotationMaxVelocity = DegreesPerSecond.of(720);
  private AngularAcceleration defaultRotationMaxAcceleration = DegreesPerSecondPerSecond.of(360);

  private static final Distance defaultPositionTolerance = Meters.of(0.025);
  private static final Distance defaultPoseOffsetX = Inches.of(1);
  private static final LinearVelocity defaultPositionVelocityTolerance = MetersPerSecond.of(0.03);
  private static final Angle defaultRotationTolerance = Degrees.of(2.0);
  private static final AngularVelocity defaultRotationVelocityTolerance = DegreesPerSecond.of(5.0);

  private double defaultDriveP_X = 0.76;
  private double defaultDriveP_Y = 0.76;
  private double defaultRotP = 0.5;

  private StructPublisher<Pose2d> targetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/DesiredPose", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> currentPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPoseFinkle/CurrentPose", Pose2d.struct)
          .publish();

  private boolean hasValidPose = false;

  private Distance positionTolerance = defaultPositionTolerance;
  private Angle rotationTolerance = defaultRotationTolerance;
  private Distance positionOffsetX = defaultPoseOffsetX;

  public DriveToPoseFinkle2(Supplier<Pose2d> desiredPoseSupplier) {
    this(
        desiredPoseSupplier,
        () -> defaultPoseOffsetX,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance);
  }

  public DriveToPoseFinkle2(Supplier<Pose2d> desiredPoseSupplier, Supplier<Distance> offset) {
    this(
        desiredPoseSupplier,
        offset,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance);
  }

  public DriveToPoseFinkle2(
      Supplier<Pose2d> desiredPoseSupplier,
      Supplier<Distance> new_positionTolerance,
      Supplier<Angle> new_rotationTolerance) {
    this(
        desiredPoseSupplier,
        () -> defaultPoseOffsetX,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance);
  }
  /**
   * Creates a new DriveToPoseFinkle command.
   *
   * @param desiredPoseSupplier
   * @param posToleranceSupplier as a Distance
   * @param new_rotationTolerance as an Angle
   */
  public DriveToPoseFinkle2(
      Supplier<Pose2d> desiredPoseSupplier,
      Supplier<Distance> offset,
      Supplier<Distance> new_positionTolerance,
      Supplier<Angle> new_rotationTolerance) {
    m_drive = RobotContainer.drivebase;

    positionTolerance = new_positionTolerance.get();
    rotationTolerance = new_rotationTolerance.get();
    positionOffsetX = offset.get();

    m_desiredPoseSupplier = desiredPoseSupplier;

    setName("DRIVETOPOSEFINKLE2");

    targetPosePub.set(new Pose2d());
    currentPosePub.set(new Pose2d());

    this.initDashboard();
    addRequirements(m_drive);
  }

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
      updatePositionValues();

      Pose2d offsetPose =
          m_desiredPoseSupplier
              .get()
              .plus(new Transform2d(positionOffsetX.in(Meters), 0, new Rotation2d()));

      targetPosePub.set(offsetPose);
      currentPosePub.set(m_pose);

      m_xDesiredPos = offsetPose.getX();
      m_yDesiredPos = offsetPose.getY();
      m_desiredRotRadians = offsetPose.getRotation().getRadians();

      Commands.print("(" + m_xDesiredPos + "  " + m_yDesiredPos + ")").schedule();

      updatePIDValues();
      updateTolerancesValues();
      updateConstraintsValues();

      // Reset the PID controllers
      xPIDController.reset(m_pose.getX());
      yPIDController.reset(m_pose.getY());
      rotPIDController.reset(m_pose.getRotation().getRadians());
      rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  @Override
  public void execute() {
    if (hasValidPose) {
      m_pose = m_drive.getPose();

      targetPosePub.set(m_desiredPoseSupplier.get());
      currentPosePub.set(m_pose);

      double calculateX = xPIDController.calculate(m_pose.getX(), m_xDesiredPos);
      double calculateY = yPIDController.calculate(m_pose.getY(), m_yDesiredPos);
      double calculateTheta =
          rotPIDController.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians);

      SmartDashboard.putNumber("Commands/DriveToPoseFinkle/CalculateX", calculateX);
      SmartDashboard.putNumber("Commands/DriveToPoseFinkle/CalculateY", calculateY);
      SmartDashboard.putNumber("Commands/DriveToPoseFinkle/CalculateTheta", calculateTheta);

      m_drive.CS_driveCommand(() -> calculateX, () -> calculateY, () -> calculateTheta, () -> true)
          .execute();

      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/ErrorX", xPIDController.getPositionError());
      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/ErrorY", yPIDController.getPositionError());
      SmartDashboard.putNumber(
          "Commands/DriveToPoseFinkle/ErrorTheta",
          Units.radiansToDegrees(rotPIDController.getPositionError()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Commodore.setCommodoreState(CommodoreState.DRIVE_FINKLE_INTERRUPTED);

    } else {
      Commodore.setCommodoreState(CommodoreState.DRIVE_FINKLE_FINISHED);
    }
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean(
        "Commands/DriveToPoseFinkle/XAtSetpoint", xPIDController.atSetpoint());
    SmartDashboard.putBoolean(
        "Commands/DriveToPoseFinkle/YAtSetpoint", yPIDController.atSetpoint());
    SmartDashboard.putBoolean(
        "Commands/DriveToPoseFinkle/RotAtSetpoint", rotPIDController.atSetpoint());

    return hasValidPose
        && (xPIDController.atSetpoint()
            && yPIDController.atSetpoint()
            && rotPIDController.atSetpoint());
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

    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/PositionTolerance(m)", positionTolerance.in(Meters));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationTolerance(deg)", rotationTolerance.in(Degrees));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/PositionVelocityTolerance(m.s-1)",
        defaultPositionVelocityTolerance.in(MetersPerSecond));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/DriveVelocityConstraint",
        defaultPositionMaxVelocity.in(MetersPerSecond));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/DriveAccelerationConstraint",
        defaultPositionMaxAcceleration.in(MetersPerSecondPerSecond));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationVelocityConstraint",
        defaultRotationMaxVelocity.in(DegreesPerSecond));
    SmartDashboard.putNumber(
        "Commands/DriveToPoseFinkle/RotationAccelerationConstraint",
        defaultRotationMaxAcceleration.in(DegreesPerSecondPerSecond));
  }

  private void updatePIDValues() {
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

    // Set the PID values
    xPIDController.setPID(drivePValueX, driveIValueX, driveDValueX);
    yPIDController.setPID(drivePValueY, driveIValueY, driveDValueY);
    rotPIDController.setPID(rotPValue, rotIValue, rotDValue);
  }

  private void updateTolerancesValues() {
    Distance newPositionTolerance =
        Meters.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/PositionTolerance(m)", positionTolerance.in(Meters)));
    Angle newRotationTolerance =
        Degrees.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/RotationTolerance(deg)",
                rotationTolerance.in(Degrees)));

    // Set the tolerances
    xPIDController.setTolerance(
        newPositionTolerance.in(Meters), defaultPositionVelocityTolerance.in(MetersPerSecond));
    yPIDController.setTolerance(
        newPositionTolerance.in(Meters), defaultPositionVelocityTolerance.in(MetersPerSecond));
    rotPIDController.setTolerance(
        newRotationTolerance.in(Radians), defaultRotationVelocityTolerance.in(RadiansPerSecond));
  }

  private void updateConstraintsValues() {
    // Max Velocity and Acceleration
    LinearVelocity newDriveMaxVelocity =
        MetersPerSecond.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/DriveVelocityConstraint",
                defaultPositionMaxVelocity.in(MetersPerSecond)));
    LinearAcceleration newDriveMaxAcceleration =
        MetersPerSecondPerSecond.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/DriveAccelerationConstraint",
                defaultPositionMaxAcceleration.in(MetersPerSecondPerSecond)));

    AngularVelocity newRotMaxVelocity =
        DegreesPerSecond.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/RotationVelocityConstraint",
                defaultRotationMaxVelocity.in(DegreesPerSecond)));
    AngularAcceleration newRotMaxAcceleration =
        DegreesPerSecondPerSecond.of(
            SmartDashboard.getNumber(
                "Commands/DriveToPoseFinkle/RotationAccelerationConstraint",
                defaultRotationMaxAcceleration.in(DegreesPerSecondPerSecond)));

    // Set the constraints for the PID controllers
    xPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            newDriveMaxVelocity.in(MetersPerSecond),
            newDriveMaxAcceleration.in(MetersPerSecondPerSecond)));
    yPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            newDriveMaxVelocity.in(MetersPerSecond),
            newDriveMaxAcceleration.in(MetersPerSecondPerSecond)));
    rotPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            newRotMaxVelocity.in(RadiansPerSecond),
            newRotMaxAcceleration.in(RadiansPerSecondPerSecond)));
  }

  private void updatePositionValues() {

    positionOffsetX =
        Inches.of(SmartDashboard.getNumber("Commands/DriveToPoseFinkle/OffsetDistance(in)", 7));
  }
}
