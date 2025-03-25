package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.UIConstants.ALGAE_FACE;

public class AlgaePreset {
  private Pose2d robotPose = null;
  private Distance elevatorHeight;
  private Angle wristAngle;
  private AngularVelocity rpm;
  public boolean hasPose = false;
  public boolean hasSubsystemsReady = false;
  private String name;

  public AlgaePreset(String name) {
    this.name = name;
    this.hasPose = false;
    this.hasSubsystemsReady = false;

    // TODO: Set Default subsystems to floor
  }

  public AlgaePreset(
      String name,
      Pose2d pose,
      Distance elevatorHeightInches,
      Angle wristAngleDegrees,
      AngularVelocity RPM) {
    this.name = name;
    this.robotPose = pose;
    this.elevatorHeight = elevatorHeightInches;
    this.wristAngle = wristAngleDegrees;
    this.rpm = RPM;
    this.hasSubsystemsReady = true;
    this.hasPose = true;
  }

  public AlgaePreset(
      String name, Distance elevatorHeightInches, Angle wristAngleDegrees, AngularVelocity RPM) {
    this.name = name;
    this.elevatorHeight = elevatorHeightInches;
    this.wristAngle = wristAngleDegrees;
    this.rpm = RPM;

    this.hasSubsystemsReady = true;
    this.hasPose = false;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPose = newPose;
    this.hasPose = true;
    System.out.printf(
        "[ALGAEPRESET] New Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
  }

  public void setSubsystems(ALGAE_FACE newPosition) {
    switch (newPosition) {
      case AB:
      case EF:
      case IJ:
        this.elevatorHeight = Presets.ALGAE_HIGH.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_HIGH.getWristAngle();
        this.rpm = Presets.ALGAE_HIGH.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case CD:
      case GH:
      case KL:
        this.elevatorHeight = Presets.ALGAE_LOW.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_LOW.getWristAngle();
        this.rpm = Presets.ALGAE_LOW.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case FLOOR:
      default:
        this.elevatorHeight = Presets.ALGAE_FLOOR.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_FLOOR.getWristAngle();
        this.rpm = Presets.ALGAE_FLOOR.getRPM();
        break;
    }

    System.out.printf(
        "[ALGAEPRESET] New Algae Preset - %s - %s In, %s Deg, %s RPM\n",
        this.toString(), elevatorHeight, wristAngle, rpm);
  }

  private void setSubsystems(String newPositionName) {
    switch (newPositionName) {
      case "AB":
      case "EF":
      case "IJ":
        this.elevatorHeight = Presets.ALGAE_HIGH.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_HIGH.getWristAngle();
        this.rpm = Presets.ALGAE_HIGH.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "CD":
      case "GH":
      case "KL":
        this.elevatorHeight = Presets.ALGAE_LOW.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_LOW.getWristAngle();
        this.rpm = Presets.ALGAE_LOW.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "PROCESSOR":
        this.elevatorHeight = Presets.ALGAE_PROCESS.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_PROCESS.getWristAngle();
        this.rpm = Presets.ALGAE_PROCESS.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "POPSTICLE":
        this.elevatorHeight = Presets.ALGAE_POPSTICLE.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_POPSTICLE.getWristAngle();
        this.rpm = Presets.ALGAE_POPSTICLE.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "FLOOR":
      default:
        this.elevatorHeight = Presets.ALGAE_FLOOR.getElevatorHeight();
        this.wristAngle = Presets.ALGAE_FLOOR.getWristAngle();
        this.rpm = Presets.ALGAE_FLOOR.getRPM();
        break;
    }

    System.out.printf(
        "[ALGAEPRESET] New Algae Preset - %s - %s In, %s Deg, %s RPM\n",
        this.toString(), elevatorHeight, wristAngle, rpm);
  }

  public AngularVelocity getRPM() {
    // Get the RPM of the Coral Shooter
    return this.rpm;
  }

  public void setRPM(AngularVelocity newRPM) {
    this.rpm = newRPM;
  }

  public void setElevatorHeightInches(Distance newElevatorHeight) {
    this.elevatorHeight = newElevatorHeight;
  }

  public Distance getElevatorHeight() {
    return this.elevatorHeight;
  }

  public void setWristAngleDegrees(Angle newWristAngle) {
    this.wristAngle = newWristAngle;
  }

  public Angle getWristAngle() {
    return this.wristAngle;
  }

  /**
   * Get the Pose of the Robot
   *
   * @return new Pose if the Pose is not set, otherwise the Pose of the Robot
   */
  public Pose2d getShootingPose() {
    Pose2d retVal = new Pose2d();

    if (hasPose && PresetManager.usingAlgaeShootDtp().getAsBoolean()) {
      retVal = this.robotPose;
    }
    return retVal;
  }

  public Pose2d getPose() {
    Pose2d retVal = new Pose2d();
    retVal = this.robotPose;
    return retVal;
  }

  /**
   * Get the Pose of the Robot
   *
   * @return new Pose if the Pose is not set, otherwise the Pose of the Robot
   */
  public Pose2d getDealgaefyPose() {
    Pose2d retVal = new Pose2d();

    if (hasPose && PresetManager.usingDealgaefyDtp().getAsBoolean()) {
      retVal = this.robotPose;
    }
    return retVal;
  }

  public void reset() {
    this.hasPose = false;

    // Reset to NONE (Floor)
    this.setSubsystems(ALGAE_FACE.FLOOR);
  }
}
