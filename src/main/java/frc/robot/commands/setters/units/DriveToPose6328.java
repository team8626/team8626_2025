package frc.robot.commands.setters.units;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.util.GeomUtil;

public class DriveToPose6328 extends CS_Command {
  private Supplier<Pose2d> desiredPoseSupplier;
  private Supplier<Pose2d> robotPoseSupplier = () -> RobotContainer.drivebase.getPose2d();
  private CS_DriveSubsystem drivebase;

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double positionErrorAbs = 0.0;
  private double rotationErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  // Tunable parameters
  private double kRotP = 0.03, kRotI = 0.0, kRotD = 0.0;
  private double kPosP = 0.03, kPosI = 0.0, kPosD = 0.0;

  private double positionMaxVelocity = 3.8;
  private double positionMaxAcceleration = 3.0;
  private double rotationMaxVelocity = Units.degreesToRadians(360.0);
  private double rotationMaxAcceleration = 8.0; // deg.s-2
  private double positionTolerance = 0.01; // meters
  private double rotationTolerance = Units.degreesToRadians(1.0);

  private double ffMinRadius = 0.05;
  private double ffMaxRadius = 0.1;

  private final ProfiledPIDController positionController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0), 0.02);
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0), 0.02);

  private StructPublisher<Pose2d> desiredPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPose/DesiredPose", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> currentPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPose/CurrentPose", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> setpointPub =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Commands/DriveToPose/setpointPose", Pose2d.struct)
          .publish();

  public DriveToPose6328(Supplier<Pose2d> desiredPoseSupplier) {
    drivebase = RobotContainer.drivebase;
    this.desiredPoseSupplier = desiredPoseSupplier;

    addRequirements(drivebase);
    this.setTAGString("DriveToPose");
    this.updateDashboard();
  }

  public DriveToPose6328(Supplier<Pose2d> desiredPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
    this(desiredPoseSupplier);
    this.robotPoseSupplier = robotPoseSupplier;
  }

  public DriveToPose6328(
      Supplier<Pose2d> desiredPoseSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> linearFFSupplier,
      DoubleSupplier omegaFFSupplier) {
    this(desiredPoseSupplier, robotPoseSupplier);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivebase.getPose2d();
    ChassisSpeeds fieldVelocity = drivebase.getRobotVelocity();
    Commodore.setCommodoreState(CommodoreState.DRIVE_AUTO);

    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    positionController.reset(
        currentPose.getTranslation().getDistance(desiredPoseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    desiredPoseSupplier
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    positionController.setTolerance(positionTolerance);
    positionController.setP(kPosP);
    positionController.setI(kPosI);
    positionController.setD(kPosD);
    positionController.setConstraints(
        new TrapezoidProfile.Constraints(positionMaxVelocity, positionMaxAcceleration));

    rotationController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    rotationController.setTolerance(rotationTolerance);
    rotationController.setP(kRotP);
    rotationController.setI(kRotI);
    rotationController.setD(kRotD);
    rotationController.setConstraints(
        new TrapezoidProfile.Constraints(rotationMaxVelocity, rotationMaxAcceleration));

    lastSetpointTranslation = currentPose.getTranslation();

    this.updateDashboard();
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = robotPoseSupplier.get();
    Pose2d desiredPose = desiredPoseSupplier.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(desiredPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    positionErrorAbs = currentDistance;

    positionController.reset(
        lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
        positionController.getSetpoint().velocity);

    double driveVelocityScalar =
        positionController.getSetpoint().velocity * ffScaler
            + positionController.calculate(positionErrorAbs, 0.0);

    if (currentDistance < positionController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                desiredPose.getTranslation(),
                currentPose.getTranslation().minus(desiredPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(positionController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double rotationVelocity =
        rotationController.getSetpoint().velocity * ffScaler
            + rotationController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

    rotationErrorAbs =
        Math.abs(currentPose.getRotation().minus(desiredPose.getRotation()).getRadians());

    if (rotationErrorAbs < rotationController.getPositionTolerance()) rotationVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(desiredPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(RobotConstants.maxLinearSpeed), linearS);
    rotationVelocity =
        MathUtil.interpolate(
            rotationVelocity, omegaFF.getAsDouble() * RobotConstants.maxAngularSpeed, thetaS);

    // Command speeds
    drivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            rotationVelocity,
            currentPose.getRotation()));

    this.updateDashboard();
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
    running = false;

    // Clear logs
    desiredPosePub.set(new Pose2d());
    setpointPub.set(new Pose2d());

    Commodore.setCommodoreState(CommodoreState.IDLE);
  }

  /** Checks if the robot is stopped at the final pose. */
  @Override
  public boolean isFinished() {
    // printf("I want: X: %3f Y: %3f Theta: %3f\n", desiredPoseSupplier.get().getX(),
    // desiredPoseSupplier.get().getY(), desiredPoseSupplier.get().getRotation().getDegrees());
    // printf("I have: X: %3f Y: %3f Theta: %3f (Pos: %b, Rot: %b)\n",
    // desiredPoseSupplier.get().getX(), desiredPoseSupplier.get().getY(),
    // desiredPoseSupplier.get().getRotation().getDegrees(), positionController.atGoal(),
    // rotationController.atGoal());
    return running && positionController.atGoal() && rotationController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(positionErrorAbs) < driveTolerance
        && Math.abs(rotationErrorAbs) < thetaTolerance.getRadians();
  }

  public void updateDashboard() {
    Pose2d currentPose = robotPoseSupplier.get();
    Pose2d desiredPose = desiredPoseSupplier.get();

    // Using SmartDashboard to tune Rotation PIDs
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Rotation/P", rotationController.getP());
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Rotation/I", rotationController.getI());
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Rotation/D", rotationController.getD());

    // Using SmartDashboard to tune Position PIDs
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Position/P", positionController.getP());
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Position/I", positionController.getI());
    SmartDashboard.putNumber("Commands/DriveToPose/Gains/Position/D", positionController.getD());

    // Display the target values
    desiredPosePub.set(desiredPose);
    currentPosePub.set(currentPose);
    setpointPub.set(
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(rotationController.getSetpoint().position)));

    // Display the tolerance values
    SmartDashboard.putNumber(
        "Commands/DriveToPose/RotationTolerance (Deg)", rotationController.getPositionTolerance());
    SmartDashboard.putNumber(
        "Commands/DriveToPose/RotationVelocityTolerance (Deg.s-1)",
        rotationController.getVelocityTolerance());
    SmartDashboard.putNumber(
        "Commands/DriveToPose/PositionTolerance (m)", positionController.getPositionTolerance());
    SmartDashboard.putNumber(
        "Commands/DriveToPose/PositionVelocityTolerance (m.s-1)",
        positionController.getVelocityTolerance());
  }
}
