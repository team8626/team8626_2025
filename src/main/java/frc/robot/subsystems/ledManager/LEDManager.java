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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.Dashboard;
import java.util.Optional;

public class LEDManager extends CS_SubsystemBase {

  // Singleton instance
  private static LEDManager instance;
  private static AddressableLEDBuffer LEDBuffer1;
  private static AddressableLED LEDs1;
  private static AddressableLEDBuffer LEDBuffer2;
  private static AddressableLED LEDs2;
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

    LEDs1 = new AddressableLED(LEDConstants.kLEDPort1);
    LEDBuffer1 = new AddressableLEDBuffer(LEDConstants.kLEDStripLength1);

    LEDs1.setLength(LEDBuffer1.getLength());

    LEDs2 = new AddressableLED(LEDConstants.kLEDPort2);
    LEDBuffer2 = new AddressableLEDBuffer(LEDConstants.kLEDStripLength2);

    LEDs2.setLength(LEDBuffer2.getLength());

    // TODO Use Constants instead of fixed values
    m_left =
        LEDBuffer1.createView(
            LEDConstants.kLEDSectionLeft.startId(), LEDConstants.kLEDSectionLeft.endId());
    m_right =
        LEDBuffer2.createView(
            LEDConstants.kLEDSectionRight.startId(), LEDConstants.kLEDSectionLeft.endId());
    m_back_bottom =
        LEDBuffer1.createView(
            LEDConstants.kLEDSectionCoral.startId(), LEDConstants.kLEDSectionCoral.endId());
    m_back_top =
        LEDBuffer2.createView(
            LEDConstants.kLEDSectionAlgae.startId(), LEDConstants.kLEDSectionAlgae.endId());

    // Apply the LED pattern to the data buffer
    LEDs1.setData(LEDBuffer1);
    LEDs2.setData(LEDBuffer2);
    LEDs1.start();
    LEDs2.start();
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
      case DISCONNECTED: // made a heart beat disconnected a white heart beat.
        currentColor = new Color[] {Color.kHotPink, Color.kPink};
        breatheSlow(currentColor).applyTo(m_left);
        breatheSlow(currentColor).applyTo(m_right);
        break;

      case DISABLED:
        // heart beat alliance
        currentColor = getAllianceColor();
        breatheSlow(currentColor).applyTo(m_left);
        breatheSlow(currentColor).applyTo(m_right);
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
        breatheSlow(Color.kCoral, Color.kBlack).applyTo(m_left);
        breatheSlow(Color.kCoral, Color.kBlack).applyTo(m_right);
        break;

      default:
        wave(currentColor).applyTo(m_left);
        wave(currentColor).applyTo(m_right);
        break;
    }
  }

  private static void updateCoralLEDs() {
    switch (Dashboard.getCoralState()) {
      case RAMPING_UP:
      case LAUNCHING:
        currentColor = new Color[] {Color.kCoral, Color.kBlack};
        breatheFast(currentColor).applyTo(m_back_top);
        break;

      case IDLE:
        LEDPattern new_pattern = LEDPattern.solid(Color.kBlack);
        new_pattern.applyTo(m_back_top);
        break;

      case INTAKING:
        currentColor = new Color[] {Color.kCoral, Color.kBlack};
        breatheSlow(currentColor).applyTo(m_back_top);
        break;

      case LOADED:
        new_pattern = LEDPattern.solid(Color.kCoral);
        new_pattern.applyTo(m_back_top);
        // soild Coral lights
        break;

      default:
        break;
    }
  }

  private static void updateAlgaeLEDs() {
    switch (Dashboard.getAlgaeState()) {
      case RAMPING_UP:
      case LAUNCHING:
        currentColor = new Color[] {Color.kAquamarine, Color.kBlack};
        breatheFast(currentColor).applyTo(m_back_bottom);
        break;

      case IDLE:
        LEDPattern new_pattern = LEDPattern.solid(Color.kBlack);
        new_pattern.applyTo(m_back_bottom);
        break;

      case INTAKING:
        currentColor = new Color[] {Color.kAquamarine, Color.kBlack};
        breatheSlow(currentColor).applyTo(m_back_bottom);
        break;

      case LOADED:
        new_pattern = LEDPattern.solid(Color.kAquamarine);
        new_pattern.applyTo(m_back_bottom);
        break;

      default:
        break;
    }
  }

  private static void updateStatusLEDs() {
    // rsl light
    boolean rsl = RobotController.getRSLState();
    LEDBuffer1.setRGB(0, 0, rsl ? 255 : 0, 0);
    LEDBuffer1.setRGB(1, 0, rsl ? 255 : 0, 0);
    LEDBuffer2.setRGB(0, 0, rsl ? 255 : 0, 0);
    LEDBuffer2.setRGB(1, 0, rsl ? 255 : 0, 0);
    // brown out
    boolean brownOut = RobotController.isBrownedOut();
    LEDBuffer1.setRGB(2, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer1.setRGB(3, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer2.setRGB(2, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer2.setRGB(3, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    // radio light
    RadioLEDState radioState = RobotController.getRadioLEDState();
    int r = 0, g = 0, b = 0;
    switch (radioState) {
      case kGreen:
        g = 255;
        break;
      case kRed:
        r = 255;
        break;
      case kOrange:
        r = 255;
        g = 165;
        break;
      case kOff:

      default:
        break;
    }
    LEDBuffer1.setRGB(4, r, g, b);
    LEDBuffer1.setRGB(5, r, g, b);
    LEDBuffer2.setRGB(4, r, g, b);
    LEDBuffer2.setRGB(5, r, g, b);
  }

  private static LEDPattern breatheFast(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .breathe(Seconds.of(.2)); // TODO: Create a constant for duration
    return new_pattern;
  }

  private static LEDPattern breatheSlow(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .breathe(Seconds.of(0.8)); // TODO: Create a constant for duration
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
    updateCoralLEDs();
    updateAlgaeLEDs();
    updateStatusLEDs();

    LEDs1.setData(LEDBuffer1);

    LEDs2.setData(LEDBuffer2);
  }

  //   public static void error(errorSections errorSections, Color color) {
  // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'error'");
  // }
}
