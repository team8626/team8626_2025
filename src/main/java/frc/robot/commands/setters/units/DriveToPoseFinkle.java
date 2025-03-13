// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.Supplier;

public class DriveToPoseFinkle extends Command {

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

  private double defaultPositionMaxVelocity = 3.8;
  private double defaultPositionMaxAcceleration = 3.0;
  private double rotationMaxVelocity = Units.degreesToRadians(360.0);
  private double rotationMaxAcceleration = 8.0; // deg.s-2
  private double positionTolerance = 0.01; // meters
  private double positionVelocityTolerance = 0.01; // meters
  private double rotationToleranceRadians = Units.degreesToRadians(1.0);
  private double rotationVelocityTolerance = Units.degreesToRadians(1.0);

  private double defaultDriveP = 0.51;
  private double defaultRotP = 0.005;

  // Will only work when atSetpoint() set
  private boolean m_finish;

  public DriveToPoseFinkle(Supplier<Pose2d> desiredPoseSupplier, boolean finish) {
    m_drive = RobotContainer.drivebase;

    m_desiredPoseSupplier = desiredPoseSupplier;

    m_finish = finish;

    addRequirements(m_drive);

    setName("TranslateToPosition");

    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Position/P",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/P", defaultDriveP));
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Position/I",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Position/D",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/D", 0.0));

    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Rotation/P",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Rotation/P", defaultRotP));
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Rotation/I",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Rotation/I", 0.0));
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/Gains/Rotation/D",
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Rotation/D", 0.0));

    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/PositionTolerance(m)", positionTolerance);
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/PositionVelocityTolerance(m.s-2)", positionVelocityTolerance);

    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/DriveVelocityConstraint", defaultPositionMaxVelocity);
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/DriveAccelerationConstraint", defaultPositionMaxAcceleration);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);

    m_pose = m_drive.getPose2d();

    m_xDesiredPos = m_desiredPoseSupplier.get().getX();
    m_yDesiredPos = m_desiredPoseSupplier.get().getY();
    m_desiredRotRadians = m_desiredPoseSupplier.get().getRotation().getRadians();

    Commands.print("(" + m_xDesiredPos + "  " + m_yDesiredPos + ")").schedule();

    double drivePValue =
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/P", defaultDriveP);
    double driveIValue =
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/I", 0);
    double driveDValue =
        SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/D", 0);

    double rotPValue = SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/P", defaultRotP);
    double rotIValue = SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/I", 0);
    double rotDValue = SmartDashboard.getNumber("Commands/TranslateToPosition/Gains/Position/D", 0);

    double newDriveMaxVelocity =
        SmartDashboard.getNumber(
            "Commands/TranslateToPosition/DriveVelocityConstraint", defaultPositionMaxVelocity);
    double newDriveMaxAcceleration =
        SmartDashboard.getNumber(
            "Commands/TranslateToPosition/DriveAccelerationConstraint",
            defaultPositionMaxAcceleration);

    m_xPID.setConstraints(
        new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));
    m_yPID.setConstraints(
        new TrapezoidProfile.Constraints(newDriveMaxVelocity, newDriveMaxAcceleration));

    m_xPID.setPID(drivePValue, driveIValue, driveDValue);
    m_yPID.setPID(drivePValue, driveIValue, driveDValue);
    m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

    m_xPID.setTolerance(positionTolerance, positionVelocityTolerance);
    m_yPID.setTolerance(positionTolerance, positionVelocityTolerance);
    m_rotPID.setTolerance(rotationToleranceRadians, rotationVelocityTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
    m_rotPID.reset(m_pose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    m_pose = m_drive.getPose2d();

    m_drive.drive(
        new ChassisSpeeds(
            m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
            m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
            m_rotPID.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians)));

    SmartDashboard.putNumber("Commands/TranslateToPosition/ErrorX", m_xPID.getPositionError());
    SmartDashboard.putNumber("Commands/TranslateToPosition/ErrorY", m_yPID.getPositionError());
    SmartDashboard.putNumber(
        "Commands/TranslateToPosition/ErrorTheta", m_rotPID.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    Commodore.setCommodoreState(CommodoreState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return m_finish && m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
  }
}
