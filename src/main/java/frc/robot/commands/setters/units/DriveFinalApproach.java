// package frc.robot.commands.setters.units;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotContainer;
// import frc.robot.commands.CS_Command;
// import java.util.function.Supplier;

// public class DriveFinalApproach extends CS_Command {
//   private Supplier<Pose2d> poseSupplier;
//   private CS_DriveSubsystem drivebase;

//   private static final double POSITION_TOLERANCE_DEGREES = 3;
//   private static final double VELOCITY_TOLERANCE_DEG_PER_SEC = 2.0;
//   private static final double POSITION_TOLERANCE_INCHES = 1;
//   private static final double VELOCITY_TOLERANCE_IN_PER_SEC = 0.25;

//   Rotation2d desiredRotation = new Rotation2d();
//   double desiredPositionX = 0;
//   double desiredPositionY = 0;

//   Rotation2d currentRotation = new Rotation2d();
//   double currentPositionX = 0;
//   double currentPositionY = 0;

//   // private double errorDegrees = 0.0;
//   // private double errorXMeters = 0.0;
//   // private double errorYMeters = 0.0;

//   private double kRotP = 0.03, kRotI = 0.0, kRotD = 0.0;
//   private double kPosP = 0.03, kPosI = 0.0, kPosD = 0.0;

//   private final TrapezoidProfile.Constraints rotationPidProfileConstraints =
//       new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);
//   private final ProfiledPIDController rotationPidController =
//       new ProfiledPIDController(kRotP, kRotI, kRotD, rotationPidProfileConstraints);

//   private final TrapezoidProfile.Constraints positionPidProfileConstraints =
//       new TrapezoidProfile.Constraints(Units.inchesToMeters(3), Units.inchesToMeters(1.5));

//   private final ProfiledPIDController positionXPidController =
//       new ProfiledPIDController(kPosP, kPosI, kPosD, positionPidProfileConstraints);
//   private final ProfiledPIDController positionYPidController =
//       new ProfiledPIDController(kPosP, kPosI, kPosD, positionPidProfileConstraints);

//   public DriveFinalApproach(Supplier<Pose2d> poseSupplier) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     // For example: addRequirements(Robot.m_subsystem);
//     drivebase = RobotContainer.drivebase;
//     this.poseSupplier = poseSupplier;

//     addRequirements(drivebase);

//     this.setTAGString("DRIVEROTATETOANGLE");
//     this.updateDashboard();
//   }

//   @Override
//   public void initialize() {
//     desiredRotation = poseSupplier.get().getRotation();
//     desiredPositionX = poseSupplier.get().getX();
//     desiredPositionY = poseSupplier.get().getY();

//     positionXPidController.setTolerance(
//         Units.inchesToMeters(POSITION_TOLERANCE_INCHES),
//         Units.inchesToMeters(VELOCITY_TOLERANCE_IN_PER_SEC));

//     positionYPidController.setTolerance(
//         Units.inchesToMeters(POSITION_TOLERANCE_INCHES),
//         Units.inchesToMeters(VELOCITY_TOLERANCE_IN_PER_SEC));

//     rotationPidController.setTolerance(
//         Units.degreesToRadians(POSITION_TOLERANCE_DEGREES),
//         Units.degreesToRadians(VELOCITY_TOLERANCE_DEG_PER_SEC));
//     rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

//     double newRotP =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/P", kRotP);
//     double newRotI =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/I", kRotI);
//     double newRotD =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Rotation/D", kRotD);

//     if (newRotP != rotationPidController.getP()) {
//       rotationPidController.setP(newRotP);
//     }
//     if (newRotI != rotationPidController.getI()) {
//       rotationPidController.setI(newRotI);
//     }
//     if (newRotD != rotationPidController.getD()) {
//       rotationPidController.setD(newRotD);
//     }

//     double newPosP =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/P", kPosP);
//     double newPosI =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/I", kPosI);
//     double newPosD =
//         SmartDashboard.getNumber("Commands/DriveRotateToAngle/Gains/Position/D", kPosD);

//     if (newPosP != positionXPidController.getP()) {
//       positionXPidController.setP(newPosP);
//       positionYPidController.setP(newPosP);
//     }
//     if (newPosI != positionXPidController.getI()) {
//       positionXPidController.setI(newPosI);
//       positionYPidController.setI(newPosI);
//     }
//     if (newPosD != positionXPidController.getD()) {
//       positionXPidController.setD(newPosD);
//       positionYPidController.setD(newPosD);
//     }

//     this.updateDashboard();
//   }

//   @Override
//   public void execute() {
//     currentRotation = drivebase.getPose2d().getRotation();
//     currentPositionX = drivebase.getPose2d().getX();
//     currentPositionY = drivebase.getPose2d().getY();

//     // errorDegrees = currentRotation.getDegrees() - desiredRotation.getDegrees();
//     // errorXMeters = currentPositionX - desiredPositionX;
//     // errorYMeters = currentPositionY - desiredPositionY;

//     drivebase.drive(
//         new ChassisSpeeds(
//             positionXPidController.calculate(drivebase.getPose2d().getX(), desiredPositionX),
//             positionYPidController.calculate(drivebase.getPose2d().getY(), desiredPositionY),
//             0));
//     // rotationPidController.calculate(
//     //     currentRotation.getRadians(), desiredRotation.getRadians())));

//     this.updateDashboard();
//   }

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     boolean retVal;

//     printf(
//         "I want x: %f, y: %f, r: %f\n",
//         desiredPositionX, desiredPositionY, desiredRotation.getDegrees());
//     printf(
//         "I am at x: %f, y: %f, r: %f\n",
//         currentPositionX, currentPositionY, currentRotation.getDegrees());
//     // return (positionXPidController.atSetpoint() && positionYPidController.atSetpoint());

//     retVal =
//         Math.abs(desiredPositionX - currentPositionX)
//                 <= Units.inchesToMeters(POSITION_TOLERANCE_INCHES)
//             && Math.abs(desiredPositionY - currentPositionY)
//                 <= Units.inchesToMeters(POSITION_TOLERANCE_INCHES);

//     return retVal;
//   }

//   public void updateDashboard() {
//     // Using SmartDashboard to tune Rotation PIDs
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Rotation/P", rotationPidController.getP());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Rotation/I", rotationPidController.getI());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Rotation/D", rotationPidController.getD());

//     // Using SmartDashboard to tune Position PIDs
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Position/P", positionXPidController.getP());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Position/I", positionXPidController.getI());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/Gains/Position/D", positionXPidController.getD());

//     // Display the target values
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/DesiredAngleDegrees", desiredRotation.getDegrees());
//     SmartDashboard.putNumber("Commands/DriveRotateToAngle/DesiredPositionX", desiredPositionX);
//     SmartDashboard.putNumber("Commands/DriveRotateToAngle/DesiredPositionY", desiredPositionY);

//     // Display the current values
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/CurrentAngleDegrees", currentRotation.getDegrees());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/CurrentPositionX", drivebase.getPose2d().getX());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/CurrentPositionY", drivebase.getPose2d().getY());

//     // Display the error values
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/ErrorDegrees",
//         Math.toDegrees(rotationPidController.getPositionError()));
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/ErrorXMeters", positionXPidController.getPositionError());
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/ErrorYMeters", positionYPidController.getPositionError());

//     // Display the tolerance values
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/RotationTolerance (Deg)", POSITION_TOLERANCE_DEGREES);
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/RotationVelocityTolerance (DegPerSec)",
//         VELOCITY_TOLERANCE_DEG_PER_SEC);
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/PositionTolerance (In)", POSITION_TOLERANCE_INCHES);
//     SmartDashboard.putNumber(
//         "Commands/DriveRotateToAngle/PosutuonVelocityTolerance (InPerSec)",
//         VELOCITY_TOLERANCE_IN_PER_SEC);
//   }
// }
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import java.util.function.Supplier;

public class DriveFinalApproach extends Command {

  CS_DriveSubsystem m_drive = RobotContainer.drivebase;

  private final ProfiledPIDController m_xPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_yPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Supplier<Pose2d> m_desiredPoseSupplier;
  private Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;

  // Will only work when atSetpoint() set
  private boolean m_finish;

  public DriveFinalApproach(Supplier<Pose2d> desiredPoseSupplier, boolean finish) {
    m_desiredPoseSupplier = desiredPoseSupplier;

    m_finish = finish;

    addRequirements(m_drive);

    setName("Drive To Pose PID Command");

    SmartDashboard.putNumber(
        "Drive Position P Value", SmartDashboard.getNumber("Drive Position P Value", 5));
    SmartDashboard.putNumber(
        "Drive Position I Value", SmartDashboard.getNumber("Drive Position I Value", 0));
    SmartDashboard.putNumber(
        "Drive Position D Value", SmartDashboard.getNumber("Drive Position D Value", 0.45));

    SmartDashboard.putNumber("Drive Velocity Constraint", 6);
    SmartDashboard.putNumber("Drive Acceleration Constraint", 4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose2d();

    m_xDesiredPos = m_desiredPoseSupplier.get().getX();
    m_yDesiredPos = m_desiredPoseSupplier.get().getY();

    Commands.print("(" + m_xDesiredPos + "  " + m_yDesiredPos + ")").schedule();

    double drivePValue = SmartDashboard.getNumber("Drive Position P Value", 1);
    double driveIValue = SmartDashboard.getNumber("Drive Position I Value", 0);
    double driveDValue = SmartDashboard.getNumber("Drive Position D Value", 0);

    double driveMaxVelocity = SmartDashboard.getNumber("Drive Velocity Constraint", 6);
    double driveMaxAcceleration = SmartDashboard.getNumber("Drive Acceleration Constraint", 4);

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));

    m_xPID.setPID(drivePValue, driveIValue, driveDValue);
    m_yPID.setPID(drivePValue, driveIValue, driveDValue);

    m_xPID.setTolerance(0.05, 0.2);
    m_yPID.setTolerance(0.05, 0.2);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pose = m_drive.getPose2d();

    m_drive.drive(
        new ChassisSpeeds(
            m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
            m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
            0));
    SmartDashboard.putNumber("X Error", m_xPID.getPositionError());
    SmartDashboard.putNumber("Y Error", m_yPID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish && m_xPID.atSetpoint() && m_yPID.atSetpoint();
  }
}
