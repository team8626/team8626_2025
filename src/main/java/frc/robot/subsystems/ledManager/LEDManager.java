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
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.subsystems.CS_SubsystemBase;
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
        wave(Color.kGreen, Color.kBlack).applyTo(m_left);
        wave(Color.kGreen, Color.kBlack).applyTo(m_right);
        break;

      case TUNE_CORALSHOOTER:
        breathe(Color.kOrange, Color.kYellow);
        break;

      default:
        wave(currentColor).applyTo(m_left);
        wave(currentColor).applyTo(m_right);
        break;
    }
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

    LEDs.setData(LEDBuffer);

    // Update the buffer with the rainbow animation

    // Set the LEDs
    LEDs.setData(LEDBuffer);
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
