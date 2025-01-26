package frc.robot.subsystems.ledManager;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class LEDConstants {
  public static final int kLEDStripLength = 143;
  public static final int kLEDPort = 9;

  record LEDSection(int startId, int endId) {}

  public static final LEDSection kSectionLeft = new LEDSection(0, 24);
  public static final LEDSection kSectionRight = new LEDSection(25, 49);
  public static final LEDSection kSectionFront = new LEDSection(50, 74);
  public static final LEDSection kSectionBack = new LEDSection(75, 99);
  public static final LEDSection kSectionAmbiance = new LEDSection(100, 142);

  public static final LinearVelocity kLEDScrollSpeed = MetersPerSecond.of(0.3);
  public static final Distance kLEDSpacing = Meters.of(0.0167);

  public static final Time kBlinkFastPeriod = Seconds.of(0.2);
  public static final Time kBlinkSlowPeriod = Seconds.of(0.5);
}
