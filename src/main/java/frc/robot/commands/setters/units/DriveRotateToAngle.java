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

/**
 * @deprecated This class is obsolete and scheduled for removal.
 */
@Deprecated
public class DriveRotateToAngle extends CS_Command {
  private Supplier<Pose2d> poseSupplier;
  private CS_DriveSubsystem drivebase;

  private static final double POSITION_TOLERANCE_DEGREES = 2.0;
  private static final double VELOCITY_TOLERANCE_DEG_PER_SEC = 2.0;
  Rotation2d desiredRotation = new Rotation2d();
  Rotation2d currentRotation = new Rotation2d();
  private double errorDegrees = 0.0;
  private double kP = 5, kI = 0.0, kD = 0.0;

  private final TrapezoidProfile.Constraints pidProfileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);
  private final ProfiledPIDController rotationPidController =
      new ProfiledPIDController(kP, kI, kD, pidProfileConstraints);

  public DriveRotateToAngle(Supplier<Pose2d> poseSupplier) {
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

    rotationPidController.setTolerance(
        Units.degreesToRadians(POSITION_TOLERANCE_DEGREES),
        Units.degreesToRadians(VELOCITY_TOLERANCE_DEG_PER_SEC));
    rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

    double newP = SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/P", kP);
    double newI = SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/I", kI);
    double newD = SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/D", kD);

    if (newP != rotationPidController.getP()) {
      rotationPidController.setP(newP);
    }
    if (newI != rotationPidController.getI()) {
      rotationPidController.setI(newI);
    }
    if (newD != rotationPidController.getD()) {
      rotationPidController.setD(newD);
    }
    this.updateDashboard();
  }

  @Override
  public void execute() {
    currentRotation = drivebase.getPose2d().getRotation();
    errorDegrees = currentRotation.getDegrees() - desiredRotation.getDegrees();

    drivebase.drive(
        new ChassisSpeeds(
            0,
            0,
            rotationPidController.calculate(
                currentRotation.getRadians(), desiredRotation.getRadians())));

    this.updateDashboard();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return rotationPidController.atSetpoint();
    // return Math.abs(errorDegrees) <= POSITION_TOLERANCE_DEGREES;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Commands/DriveRotateToAngle/Gains/P", rotationPidController.getP());
    SmartDashboard.putNumber("Commands/DriveRotateToAngle/Gains/I", rotationPidController.getI());
    SmartDashboard.putNumber("Commands/DriveRotateToAngle/Gains/D", rotationPidController.getD());

    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/DesiredAngleDegrees", desiredRotation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/CurrentAngleDegrees", currentRotation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/ErrorDegrees",
        Math.toDegrees(rotationPidController.getPositionError()));
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/PositionTolerance (Deg)", POSITION_TOLERANCE_DEGREES);
    SmartDashboard.putNumber(
        "Commands/DriveRotateToAngle/VelocityTolerance (DegPerSec)",
        VELOCITY_TOLERANCE_DEG_PER_SEC);
  }
}
