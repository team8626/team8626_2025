package frc.robot.subsystems.ledManager;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class LEDConstants {
  public static final int kLEDStripLength = 143;
  public static final int kLEDPort = 9;

  record LEDSection(int startId, int endId) {}

  public static final LEDSection kSectionMain = new LEDSection(0, 74);
  public static final LEDSection kSectionAmbience = new LEDSection(75, 142);

  public static final Distance LedSpacing = Meters.of(1 / 144);

  public static enum errorSections {
    FRONT_RIGHT(0, new int[] {0, 1, 2, 3, 4}),
    BACK_RIGHT(1, new int[] {25, 26, 27, 28, 29}),
    BACK_LEFT(2, new int[] {45, 46, 47, 48, 49}),
    FRONT_LEFT(3, new int[] {70, 71, 72, 73, 74});

    private final int id;
    private final int[] indexes;

    errorSections(int id, int[] indexes) {
      this.id = id;
      this.indexes = indexes;
    }

    public int getValue() {
      return id;
    }

    public int[] getIndexes() {
      return indexes;
    }
  }

  public static final int minLoopCycleCount = 10;
  public static final int length = 10;
  public static final double strobeFastDuration = 0.1;
  public static final double strobeSlowDuration = 0.2;
  public static final double breathDuration = 2.0;
  public static final int rainbowSaturation = 255;
  public static final int rainbowValue = 128;
  public static final double waveExponent = 0.4;
  public static final double waveFastCycleLength = 25.0;
  public static final double waveFastDuration = 0.25;
  public static final double waveSlowCycleLength = 25.0;
  public static final double waveSlowDuration = 3.0;
  public static final double waveAllianceCycleLength = 15.0;
  public static final double waveAllianceDuration = 2.0;
  public static final double autoFadeTime = 2.5; // 3s nominal
  public static final double autoFadeMaxTime = 5.0;
}
