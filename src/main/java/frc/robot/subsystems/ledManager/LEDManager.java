// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ledManager;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.ledManager.LEDConstants.LEDSection;
import frc.robot.subsystems.ledManager.LEDConstants.errorSections;
import java.util.Optional;

public class LEDManager extends CS_SubsystemBase {

  // Singleton instance
  private static LEDManager instance;

  public static enum LedAmbienceMode {
    RAINBOW,
    OFF
  }

  public static enum LedErrorMode {
    NO_ERROR,
    ERROR_CRITICAL,
    ERROR_DRIVE_FL,
    ERROR_DRIVE_FR,
    ERROR_DRIVE_BL,
    ERROR_DRIVE_BR,
    ERROR_GIMME_LIGHT
  }

  /** Creates a new LEDSubsystem. */
  private static AddressableLED LEDs;

  private static AddressableLEDBuffer buffer;

  // static LedMode mainMode = LedMode.OFF;
  static CommodoreState mainMode = CommodoreState.IDLE;
  static LedErrorMode errorMode = LedErrorMode.NO_ERROR;
  static LedAmbienceMode ambienceMode = LedAmbienceMode.OFF;

  static Color currentColor[] = {Color.kHotPink, Color.kPink};

  private LEDManager() {
    super();

    LEDs = new AddressableLED(LEDConstants.kLEDPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);
    LEDs.setLength(LEDConstants.kLEDStripLength);
    LEDs.setData(buffer);
    LEDs.start();

    // Set the default modes
    // setMode(LedMode.NOT_CONNECTED);
    setAmbienceMode(LedAmbienceMode.OFF);
    setErrorMode(LedErrorMode.NO_ERROR);
  }

  // Public method to provide access to the singleton instance
  public static LEDManager getInstance() {
    if (instance == null) {
      instance = new LEDManager();
    }
    return instance;
  }

  /**
   * Set the mode of the Main LED sections
   *
   * @param newMode mode to set the LEDs to
   */
  // public static void setMode(LedMode newMode) {
  //   mainMode = newMode;
  //   updateMainLeds();
  // }

  /**
   * Set the mode of the Error LED sections
   *
   * @param newErrorCode mode to set the error LEDs to
   */
  public static void setErrorMode(LedErrorMode newErrorCode) {
    errorMode = newErrorCode;
    updateErrorLeds();
  }

  /**
   * Set the mode of the Ambience LED section
   *
   * @param newAmbienceCode mode to set the ambience LEDs to
   */
  public static void setAmbienceMode(LedAmbienceMode newAmbienceCode) {
    ambienceMode = newAmbienceCode;
    updateAmbienceLeds();
  }

  /**
   * Set color of the complete LED Buffer to color
   *
   * @param color
   */
  private void setColor(LEDSection section, Color color) {
    for (int i = section.startId(); i <= section.endId(); i++) {
      buffer.setLED(i, color);
    }
  }

  /** Update the main LED sections based on current set mode */
  private static void updateMainLeds() {
    mainMode = Commodore.getCurrentState();

    switch (mainMode) {
      case DISCONNECTED:
        currentColor = new Color[] {Color.kHotPink, Color.kPink};
        breath(
            LEDConstants.kSectionMain,
            currentColor[0],
            currentColor[1],
            LEDConstants.breathDuration,
            Timer.getFPGATimestamp());
        break;

      case DISABLED:
        currentColor = getAllianceColor();
        // currentColor = new Color[] {Color.kGreen, Color.kGreenYellow};
        breath(
            LEDConstants.kSectionMain,
            currentColor[0],
            currentColor[1],
            LEDConstants.breathDuration,
            Timer.getFPGATimestamp());
        break;

      case IDLE:
        currentColor = getAllianceColor();
        wave(LEDConstants.kSectionMain, currentColor[0], currentColor[1], 25, 2.0);
        break;

      case ESTOP:
        currentColor = new Color[] {Color.kGreen, Color.kGreenYellow};
        breath(
            LEDConstants.kSectionMain,
            currentColor[0],
            currentColor[1],
            LEDConstants.breathDuration,
            Timer.getFPGATimestamp());
        break;

      case UNKNOWN:
      case TRANSITION:
      case SHOOT:
        wave(LEDConstants.kSectionMain, Color.kGreen, Color.kBlack, 25, 2.0);
        break;
      case CORAL_SHOOT:
      case CORAL_SHOOT_RAMPINGUP:
        blink(LEDConstants.kSectionMain, Color.kBlueViolet, 0.6);
        break;
      case CORAL_SHOOT_LAUNCHING:
        blink(LEDConstants.kSectionMain, Color.kBlueViolet, 0.2);
        break;

      case CORAL_INTAKE:
        wave(LEDConstants.kSectionMain, Color.kAquamarine, Color.kBlack, 25, 2.0);
        break;

      case INTAKE:
        wave(LEDConstants.kSectionMain, Color.kOrange, Color.kBlack, 25, 2.0);
        break;

      case TUNE_CORALSHOOTER:
        blink(LEDConstants.kSectionMain, Color.kOrange, 0.5);
        break;


      case ERROR_CRITICAL:
        pulse(LEDConstants.kSectionMain, Color.kYellowGreen, 0.5, 2.0);
        break;

      default:
        wave(LEDConstants.kSectionMain, currentColor[0], currentColor[1], 25, 2.0);
        break;
    }
  }

  /** Update LEDs based on current set state */
  private static void updateAmbienceLeds() {
    switch (ambienceMode) {
      case OFF:
        solid(LEDConstants.kSectionAmbience, Color.kBlack);
        break;

      case RAINBOW:
      default:
        rainbow(LEDConstants.kSectionAmbience, 25, 2.0);
        break;
    }
  }

  /** Update ErrorLEDs based on current set state */
  private static void updateErrorLeds() {

    switch (errorMode) {
      case ERROR_CRITICAL:
        error(Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FL:
        error(errorSections.FRONT_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FR:
        error(errorSections.FRONT_RIGHT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BL:
        error(errorSections.BACK_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BR:
        error(errorSections.BACK_RIGHT, Color.kYellowGreen);
        break;
      case ERROR_GIMME_LIGHT:
        errorSolid(errorSections.BACK_LEFT, Color.kWhite);
        errorSolid(errorSections.BACK_RIGHT, Color.kWhite);

        break;
      case NO_ERROR:
      default:
        // Do Nothing
        break;
    }
  }

  private static void solid(LEDSection section, Color c1) {
    for (int i = section.startId(); i <= section.endId(); i++) {
      buffer.setLED(i, c1);
    }
  }

  private static void blink(LEDSection section, Color c1, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? c1 : Color.kBlack);
  }

  private static void flow(LEDSection section, Color c1, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio));
    double green = (c1.green * (1 - ratio));
    double blue = (c1.blue * (1 - ratio));
    solid(section, new Color(red, green, blue));
  }

  private static void breath(
      LEDSection section, Color c1, Color c2, double duration, double timestamp) {
    double x =
        ((timestamp % LEDConstants.breathDuration) / LEDConstants.breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  /**
   * Pulsing pattern of the LEDs
   *
   * @param c1 Color to be used
   * @param d1 Duration of the pulsing cycle (in seconds)
   * @param cycles Number of Pulses in each cycle
   */
  private static void pulse(LEDSection section, Color c1, double d1, double cycles) {
    boolean on = ((Timer.getFPGATimestamp() % (d1 * 2 * cycles)) / (d1 * 2 * cycles)) > 0.5;
    double red = 0, green = 0, blue = 0;

    if (on) {
      double x = (1 - ((Timer.getFPGATimestamp() % d1) / d1)) * Math.PI;
      // double ratio = (Math.sin(x) + 1.0);
      double ratio = 1 / x;

      red = (c1.red * (1 - ratio));
      green = (c1.green * (1 - ratio));
      blue = (c1.blue * (1 - ratio));
    }
    solid(section, new Color(red, green, blue));
  }

  /**
   * Wave pattern of the LEDs
   *
   * @param section Section of the LEDs to be used
   * @param c1 Primary color
   * @param c2 Secondary color
   * @param cycleLength number of LEDs in a cycle
   * @param duration Duration of the cycle in seconds
   */
  private static void wave(
      LEDSection section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;

    for (int i = section.startId(); i <= section.endId(); i++) {
      x += xDiffPerLed;
      // if (i >= LEDConstants.kLEDLength){
      double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
      // LEDs.setData(buffer);
    }
  }

  /**
   * Rainbow pattern of the LEDs
   *
   * @param section Section of the LEDs to be used
   * @param cycleLength number of LEDs in a cycle
   * @param duration Duration of the cycle in seconds
   */
  private static void rainbow(LEDSection section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.startId(); i <= section.endId(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  /**
   * Set color of the all Error LEDs
   *
   * @param color
   */
  private static void error(Color color) {
    for (errorSections section : errorSections.values()) {
      error(section, color);
    }
  }

  /**
   * Set color of the specified Error Section
   *
   * @param section
   * @param color
   */
  public static void error(errorSections section, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;
    for (int j = 0; j < section.getIndexes().length; j++) {
      int ledIndex = section.getIndexes()[j];
      if (on) {
        buffer.setLED(ledIndex, color);
      } else {
        buffer.setLED(ledIndex, Color.kBlack);
      }
    }
  }

  /**
   * Set color of the specified Error Section
   *
   * @param section
   * @param color
   */
  public static void errorSolid(errorSections section, Color color) {
    for (int j = 0; j < section.getIndexes().length; j++) {
      int ledIndex = section.getIndexes()[j];
      buffer.setLED(ledIndex, color);
    }
  }

  /**
   * Set color of the specified Error Sections
   *
   * @param sections
   * @param color
   */
  private void error(errorSections[] sections, Color color) {
    for (int i = 0; i < sections.length; i++) {
      LEDManager.error(sections[i], color);
    }
  }

  private static Color[] getAllianceColor() {
    Color[] newColor = new Color[] {Color.kHotPink, Color.kPink};

    // Set the color based on the alliance
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        newColor = new Color[] {Color.kRed, Color.kBlack};
      }
      if (ally.get() == Alliance.Blue) {
        newColor = new Color[] {Color.kNavy, Color.kBlack};
      }
    }
    return newColor;
  }

  @Override
  public void CS_periodic() {
    // Update the LEDs
    updateMainLeds();
    updateAmbienceLeds();
    updateErrorLeds();

    LEDs.setData(buffer);
  }

  // public Command setModeCommand(LedMode newMode) {
  //   return startEnd(() -> setMode(newMode), () -> setMode(LedMode.DEFAULT))
  //       .withName("[LEDManager] SetMode");
  // }

  public Command setErrorModeCommand(LedErrorMode newMode) {
    return startEnd(() -> setErrorMode(newMode), () -> setErrorMode(LedErrorMode.NO_ERROR))
        .withName("[LEDManager] SetErrorMode");
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {}
}
