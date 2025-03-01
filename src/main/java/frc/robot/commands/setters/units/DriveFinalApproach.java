package frc.robot.commands.setters.units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.Supplier;

public class DriveFinalApproach extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private CS_DriveSubsystem drivebase;

  private static final double POSITION_TOLERANCE_DEGREES = 2.0;
  private static final double VELOCITY_TOLERANCE_DEG_PER_SEC = 2.0;
  private static final double POSITION_TOLERANCE_INCHES = 0.5;
  private static final double VELOCITY_TOLERANCE_IN_PER_SEC = 0.25;

  Rotation2d desiredRotation = new Rotation2d();
  double desiredPositionX = 0;
  double desiredPositionY = 0;

  Rotation2d currentRotation = new Rotation2d();
  double currentPositionX = 0;
  double currentPositionY = 0;

  // private double errorDegrees = 0.0;
  // private double errorXMeters = 0.0;
  // private double errorYMeters = 0.0;

  private double kRotP = 5, kRotI = 0.0, kRotD = 0.0;
  private double kPosP = 5, kPosI = 0.0, kPosD = 0.0;

  private final TrapezoidProfile.Constraints rotationPidProfileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);
  private final ProfiledPIDController rotationPidController =
      new ProfiledPIDController(kRotP, kRotI, kRotD, rotationPidProfileConstraints);

  private final TrapezoidProfile.Constraints positionPidProfileConstraints =
      new TrapezoidProfile.Constraints(Units.inchesToMeters(3), Units.inchesToMeters(1.5));

  private final ProfiledPIDController positionXPidController =
      new ProfiledPIDController(kPosP, kPosI, kPosD, positionPidProfileConstraints);
  private final ProfiledPIDController positionYPidController =
      new ProfiledPIDController(kPosP, kPosI, kPosD, positionPidProfileConstraints);

  public DriveFinalApproach(Supplier<Pose2d> poseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    drivebase = RobotContainer.drivebase;
    this.poseSupplier = poseSupplier;

    addRequirements(drivebase);

    this.setTAGString("DRIVEROTATETOANGLE");
    this.updateDashboard();
  }

  @Override
  public void initialize() {
    desiredRotation = poseSupplier.get().getRotation();
    desiredPositionX = poseSupplier.get().getX();
    desiredPositionY = poseSupplier.get().getY();

    positionXPidController.setTolerance(
        Units.inchesToMeters(POSITION_TOLERANCE_INCHES),
        Units.inchesToMeters(VELOCITY_TOLERANCE_IN_PER_SEC));

    positionYPidController.setTolerance(
        Units.inchesToMeters(POSITION_TOLERANCE_INCHES),
        Units.inchesToMeters(VELOCITY_TOLERANCE_IN_PER_SEC));

    rotationPidController.setTolerance(
        Units.degreesToRadians(POSITION_TOLERANCE_DEGREES),
        Units.degreesToRadians(VELOCITY_TOLERANCE_DEG_PER_SEC));
    rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

    double newRotP =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/P", kRotP);
    double newRotI =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/I", kRotI);
    double newRotD =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/D", kRotD);

    if (newRotP != rotationPidController.getP()) {
      rotationPidController.setP(newRotP);
    }
    if (newRotI != rotationPidController.getI()) {
      rotationPidController.setI(newRotI);
    }
    if (newRotD != rotationPidController.getD()) {
      rotationPidController.setD(newRotD);
    }

    double newPosP =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/P", kPosP);
    double newPosI =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/I", kPosI);
    double newPosD =
        SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/D", kPosD);

    if (newPosP != positionXPidController.getP()) {
      positionXPidController.setP(newPosP);
      positionYPidController.setP(newPosP);
    }
    if (newPosI != positionXPidController.getI()) {
      positionXPidController.setI(newPosI);
      positionYPidController.setI(newPosI);
    }
    if (newPosD != positionXPidController.getD()) {
      positionXPidController.setD(newPosD);
      positionYPidController.setD(newPosD);
    }

    this.updateDashboard();
  }

  @Override
  public void execute() {
    currentRotation = drivebase.getPose2d().getRotation();
    currentPositionX = drivebase.getPose2d().getX();
    currentPositionY = drivebase.getPose2d().getY();

    // errorDegrees = currentRotation.getDegrees() - desiredRotation.getDegrees();
    // errorXMeters = currentPositionX - desiredPositionX;
    // errorYMeters = currentPositionY - desiredPositionY;

    drivebase.drive(
        new ChassisSpeeds(
            positionXPidController.calculate(drivebase.getPose2d().getX(), desiredPositionX),
            positionYPidController.calculate(drivebase.getPose2d().getY(), desiredPositionY),
            rotationPidController.calculate(
                currentRotation.getRadians(), desiredRotation.getRadians())));

    this.updateDashboard();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (rotationPidController.atSetpoint()
        && positionXPidController.atSetpoint()
        && positionYPidController.atSetpoint());
  }

  public void updateDashboard() {
    // Using SmartDashboard to tune Rotation PIDs
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Rotation/P", rotationPidController.getP());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Rotation/I", rotationPidController.getI());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Rotation/D", rotationPidController.getD());

    // Using SmartDashboard to tune Position PIDs
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Position/P", positionXPidController.getP());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Position/I", positionXPidController.getI());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/Gains/Position/D", positionXPidController.getD());

    // Display the target values
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/DesiredAngleDegrees", desiredRotation.getDegrees());
    SmartDashboard.putNumber("Commands/DriveRotateToAngle/DesiredPositionX", desiredPositionX);
    SmartDashboard.putNumber("Commands/DriveRotateToAngle/DesiredPositionY", desiredPositionY);

    // Display the current values
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/CurrentAngleDegrees", currentRotation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/CurrentPositionX", drivebase.getPose2d().getX());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/CurrentPositionY", drivebase.getPose2d().getY());

    // Display the error values
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/ErrorDegrees",
        Math.toDegrees(rotationPidController.getPositionError()));
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/ErrorXMeters", positionXPidController.getPositionError());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/ErrorYMeters", positionYPidController.getPositionError());

    // Display the tolerance values
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/RotationTolerance (Deg)", POSITION_TOLERANCE_DEGREES);
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/RotationVelocityTolerance (DegPerSec)",
        VELOCITY_TOLERANCE_DEG_PER_SEC);
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/PositionTolerance (In)", POSITION_TOLERANCE_INCHES);
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/PosutuonVelocityTolerance (InPerSec)",
        VELOCITY_TOLERANCE_IN_PER_SEC);
  }
}
