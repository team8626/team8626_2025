package frc.robot.subsystems.ledManager;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class LEDConstants {
  public static final int kLEDStripLength = 250;
  public static final int kLEDPort = 1;

  record LEDSection(int startId, int endId) {}

  public static final LEDSection kLEDSectionStatusLeft = new LEDSection(0, 5);
  public static final LEDSection kLEDSectionLeft = new LEDSection(6, 68);
  public static final LEDSection kLEDSectionCoral = new LEDSection(69, 122);
  public static final LEDSection kLEDSectionAlgae = new LEDSection(123, 176);
  public static final LEDSection kLEDSectionRight = new LEDSection(177, 239);
  public static final LEDSection kLEDSectionStatusRight = new LEDSection(240, 245);

  public static final Distance LedSpacing = Meters.of(1.0 / 144.0);

  public static final int rainbowSaturation = 255;
  public static final int rainbowValue = 255;
}
