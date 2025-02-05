// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ledManager;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
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
  private static AddressableLEDBuffer LEDBuffer;
  private static AddressableLED LEDs;

  private static AddressableLEDBufferView m_left;
  private static AddressableLEDBufferView m_right;
  private static AddressableLEDBufferView m_back_top;
  private static AddressableLEDBufferView m_back_bottom;

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

  // static LedMode mainMode = LedMode.OFF;

  static CommodoreState mainMode = CommodoreState.IDLE;

  static LedErrorMode errorMode = LedErrorMode.NO_ERROR;

  static Color currentColor[] = {Color.kHotPink, Color.kPink};

  private LEDManager() {

    super();

    LEDs = new AddressableLED(LEDConstants.kLEDPort);
    LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);

    LEDs.setLength(LEDBuffer.getLength());

    // TODO Use Constants instead of fixed values
    m_left = LEDBuffer.createView(0, 59);
    m_right = LEDBuffer.createView(60, 119).reversed();
    m_back_bottom = LEDBuffer.createView(120, 129);
    m_back_top = LEDBuffer.createView(130, 139);

    // Apply the LED pattern to the data buffer
    LEDs.setData(LEDBuffer);
    LEDs.start();
  }

  // Public method to provide access to the singleton instance
  public static LEDManager getInstance() {
    if (instance == null) {
      instance = new LEDManager();
    }
    return instance;
  }

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
   * Set color of the complete LED Buffer to color
   *
   * @param color
   */
  private void setColor(LEDSection section, Color color) {
    for (int i = section.startId(); i <= section.endId(); i++) {
      LEDBuffer.setLED(i, color);
    }
  }

  /** Update the main LED sections based on current set mode */
  private static void updateMainLeds() {
    mainMode = Commodore.getCurrentState();

    switch (mainMode) {
      case DISCONNECTED:
        currentColor = new Color[] {Color.kHotPink, Color.kPink};
        breathe(currentColor).applyTo(m_left);
        breathe(currentColor).applyTo(m_right);
        break;

      case DISABLED:
        currentColor = getAllianceColor();

        breathe(currentColor).applyTo(m_left);
        breathe(currentColor).applyTo(m_right);
        break;

      case IDLE:
        currentColor = getAllianceColor();

        wave(currentColor).applyTo(m_left);
        wave(currentColor).applyTo(m_right);
        break;

      case ESTOP:
        currentColor = new Color[] {Color.kGreen, Color.kGreenYellow};
        // breathe(
        //     LEDConstants.kSectionMain,
        //     currentColor[0],
        //     currentColor[1],
        //     LEDConstants.breathDuration);
        break;

      case UNKNOWN:
      case TRANSITION:
      case SHOOT:
        wave(Color.kGreen, Color.kBlack);
        break;
      case CORAL_SHOOT:
      case CORAL_SHOOT_RAMPINGUP:
        blink(LEDConstants.kSectionMain, Color.kBlueViolet, 0.6);
        break;
      case CORAL_SHOOT_LAUNCHING:
        blink(LEDConstants.kSectionMain, Color.kBlueViolet, 0.2);
        break;

      case CORAL_INTAKE:
        wave(Color.kAquamarine, Color.kBlack);
        break;

      case INTAKE:
        wave(Color.kOrange, Color.kBlack);
        break;

      case TUNE_CORALSHOOTER:
        blink(LEDConstants.kSectionMain, Color.kOrange, 0.5);
        break;

        // case ERROR_CRITICAL:
        //   pulse(LEDConstants.kSectionMain, Color.kYellowGreen, 0.5, 2.0);
        //   break;

      default:
        wave(currentColor[0], currentColor[1]);
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
      LEDBuffer.setLED(i, c1);
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

  private static LEDPattern breathe(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .breathe(Seconds.of(2)); // TODO: Create a constant for duration
    return new_pattern;
  }

  private static LEDPattern wave(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), LEDConstants.LedSpacing);
    // TODO: Create a constant for duration
    return new_pattern;
  }

  private static LEDPattern rainbow() {

    LEDPattern rainbow =
        LEDPattern.rainbow(LEDConstants.rainbowSaturation, LEDConstants.rainbowValue)
            .scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), LEDConstants.LedSpacing);

    return rainbow;
  }

  /**
   * Rainbow pattern of the LEDs
   *
   * @param section Section of the LEDs to be used
   * @param cycleLength number of LEDs in a cycle
   * @param duration Duration of the cycle in seconds /* * /** Set color of the all Error LEDs
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
        LEDBuffer.setLED(ledIndex, color);
      } else {
        LEDBuffer.setLED(ledIndex, Color.kBlack);
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
      LEDBuffer.setLED(ledIndex, color);
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
    updateErrorLeds();

    LEDs.setData(LEDBuffer);

    // Update the buffer with the rainbow animation

    // Set the LEDs
    LEDs.setData(LEDBuffer);
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

  //   public static void error(errorSections errorSections, Color color) {
  // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'error'");
  // }
}
