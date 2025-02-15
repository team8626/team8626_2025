package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;

public class Presets {
  // Will be finished later when we have Algae and Coral Locations Set
  // public static final Preset kStart = new Preset("START", "CORAL", 180, 0);
  // public static final Preset kStow = new Preset("STOW", "CORAL", 195, 0);
  // public static final Preset kCoralL4 = new Preset("CORAL L4", "CORAL", 0, 0, 0, 1275);
  // public static final Preset kCoralL1 = new Preset("CORAL L1", "CORAL", 0, 0, 0, 500);
  // public static final Preset kSourcePickup = new Preset("SOURCE PICKUP", "CORAL", 127, 11);
  // public static final Preset kAReeflgaePickup = new Preset("REEF PICKUP", "ALGAE", 127, 11);
  // public static final Preset kReefGroundPickup = new Preset("GROUND PICKUP", "ALGAE", 127, 11);
  // public static final Preset kClimbPreset = new Preset("CLIMB PRESET", "EMPTY", 80, 0);
  // public static final Preset kClimbEnd = new Preset("CLIMB FINISH", "EMPTY", 198, 0);

  public class CoralPreset {
    public String name = "'";
    public Pose2d robotPose = new Pose2d();
    public double coralRPM = 0;
    public boolean isReady = false;

    public CoralPreset(String name, Pose2d robotPose, double coralRPM, boolean isReady) {
      this.name = name;
      this.robotPose = robotPose;
      this.coralRPM = coralRPM;
      this.isReady = isReady;
    }

    public void setRPM(double newRPM) {
      // Set the RPM of the Coral Shooter
      this.coralRPM = newRPM;
    }

    public void setPose(Pose2d newPose) {
      // Set the Pose of the Robot
      this.robotPose = newPose;
    }

    public void setReady(boolean ready) {
      // Set the Ready State of the Preset
      this.isReady = ready;
    }

    public boolean isReady() {
      // Get the Ready State of the Preset
      return this.isReady;
    }
  }

  public record AlgaePreset(
      String name,
      Pose2d robotPose,
      double elevatorHeightInches,
      double wristAngleDegrees,
      double algaeRPM,
      boolean isReady) {}
}
