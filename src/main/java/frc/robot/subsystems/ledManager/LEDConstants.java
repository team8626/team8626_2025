package frc.robot.subsystems.ledManager;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ledManager.LEDConstants.LEDSection;

public class LEDConstants {
  public static final int kLEDStripLength = 247;
  public static final int kLEDPort = 1;

  record LEDSection(int startId, int endId) {}

  public static final Time breathSlowDuration = Seconds.of(.8);
  public static final Time breathFastDuration = Seconds.of(.2);
  public static final Time flashDuration = Seconds.of(.1);
  public static final LinearVelocity scrollVelocity = Centimeters.per(Second).of(6);

  // public static final LEDSection kLEDSectionStatusLeft = new LEDSection(0, 5);
  // public static final LEDSection kLEDSectionLeft = new LEDSection(6, 68);
  // public static final LEDSection kLEDSectionCoral = new LEDSection(69, 122);
  // public static final LEDSection kLEDSectionAlgae = new LEDSection(123, 176);
  // public static final LEDSection kLEDSectionRight = new LEDSection(177, 239);
  // public static final LEDSection kLEDSectionStatusRight = new LEDSection(240, 245);

  public static final LEDSection kLEDSectionStatusLeft = new LEDSection(62, 67);
  public static final LEDSection kLEDSectionLeft = new LEDSection(0, 67);
  public static final LEDSection kLEDSectionCoral = new LEDSection(68, 109);
  public static final LEDSection kLEDSectionAlgae = new LEDSection(110, 151);
  public static final LEDSection kLEDSectionRight = new LEDSection(152, 218);
  public static final LEDSection kLEDSectionStatusRight = new LEDSection(179, 180);

  public static final Distance ledSpacing = Meters.of(1.0 / 144.0); // 144 LEDs per Meter

  public static final int rainbowSaturation = 255;
  public static final int rainbowValue = 127;

  public static final Color CS_Pink = new Color("#FF69B4");
  public static final Color CS_DarkPink = new Color("#990033");

  public static final Color[] CS_Pinks = {CS_Pink, CS_DarkPink};
}
