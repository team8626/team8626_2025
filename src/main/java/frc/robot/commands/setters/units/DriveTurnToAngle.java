// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.Supplier;

public class DriveTurnToAngle extends CS_Command {

  private final SwerveSubsystem drive;

  private final ProfiledPIDController rotPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Supplier<Pose2d> m_desiredPoseSupplier;
  private Pose2d m_pose;

  private Supplier<Angle> desiredAngle = () -> Degrees.of(0);

  private AngularVelocity defaultRotationMaxVelocity = DegreesPerSecond.of(720);
  private AngularAcceleration defaultRotationMaxAcceleration = DegreesPerSecondPerSecond.of(360);

  private static final Angle defaultRotationTolerance = Degrees.of(2.0);
  private static final AngularVelocity defaultRotationVelocityTolerance = DegreesPerSecond.of(5.0);

  private double defaultRotP = 5;

  private Supplier<Angle> rotationTolerance = () -> defaultRotationTolerance;

  public DriveTurnToAngle(Supplier<Angle> desiredAngle) {
    this(desiredAngle, () -> defaultRotationTolerance);
  }

  /**
   * Creates a new DriveToPoseFinkle command.
   *
   * @param desiredAngle as an Angle
   * @param rotationTolerance as an Angle
   */
  public DriveTurnToAngle(Supplier<Angle> new_desiredAngle, Supplier<Angle> new_rotationTolerance) {
    drive = RobotContainer.drivebase;

    desiredAngle = new_desiredAngle;
    rotationTolerance = new_rotationTolerance;

    setName("TURNTOANGLE");
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // m_drive.resetOdometry();

    Commodore.setCommodoreState(CommodoreState.DRIVE_FINKLE);

    rotPIDController.setPID(defaultRotP, 0, 0);
    rotPIDController.setTolerance(
        rotationTolerance.get().in(Radians), defaultRotationVelocityTolerance.in(RadiansPerSecond));
    rotPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            defaultRotationMaxVelocity.in(RadiansPerSecond),
            defaultRotationMaxAcceleration.in(RadiansPerSecondPerSecond)));

    // Reset the PID controllers
    rotPIDController.reset(m_pose.getRotation().getRadians());
    rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    double currentRotationRadians = drive.getPose().getRotation().getRadians();

    double calculateTheta =
        rotPIDController.calculate(currentRotationRadians, desiredAngle.get().in(Radians));

    drive.CS_driveCommand(() -> 0, () -> 0, () -> calculateTheta, () -> true).execute();
  }

  @Override
  public void end(boolean interrupted) {
    Commodore.setCommodoreState(CommodoreState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return rotPIDController.atSetpoint();
  }
}
