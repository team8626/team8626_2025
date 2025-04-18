// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ledManager;

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
import java.util.Map;
import java.util.Optional;

public class LEDManager extends CS_SubsystemBase {

  // Singleton instance
  private static LEDManager instance;
  private static AddressableLED LEDs;
  private static AddressableLEDBuffer LEDBuffer;

  private static AddressableLEDBufferView m_left;
  private static AddressableLEDBufferView m_right;
  private static AddressableLEDBufferView m_back_top;
  private static AddressableLEDBufferView m_back_bottom;

  static CommodoreState mainMode = CommodoreState.IDLE;
  static Color currentColor[] = {Color.kHotPink, Color.kPink};

  private LEDManager() {

    super();

    LEDs = new AddressableLED(LEDConstants.kLEDPort);
    LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);
    LEDs.setLength(LEDBuffer.getLength());

    m_left =
        LEDBuffer.createView(
            LEDConstants.kLEDSectionLeft.startId(), LEDConstants.kLEDSectionLeft.endId());
    m_right =
        LEDBuffer.createView(
                LEDConstants.kLEDSectionRight.startId(), LEDConstants.kLEDSectionRight.endId())
            .reversed();
    m_back_bottom =
        LEDBuffer.createView(
            LEDConstants.kLEDSectionCoral.startId(), LEDConstants.kLEDSectionCoral.endId());
    m_back_top =
        LEDBuffer.createView(
            LEDConstants.kLEDSectionAlgae.startId(), LEDConstants.kLEDSectionAlgae.endId());

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
      case DISCONNECTED: // made a heart beat disconnected a white heart beat.
        currentColor = LEDConstants.CS_Pinks;
        breatheSlow(currentColor).applyTo(m_left);
        breatheSlow(currentColor).applyTo(m_right);
        break;

      case DISABLED:
        // heart beat alliance
        currentColor = getAllianceColor();
        breatheSlow(currentColor).applyTo(m_left);
        breatheSlow(currentColor).applyTo(m_right);
        break;

      case SUBSYSTEMS_ADJUST:
        currentColor = getAllianceColor();
        flash(currentColor).applyTo(m_left);
        flash(currentColor).applyTo(m_right);
        break;

      case DRIVE_AUTO:
        rainbow().applyTo(m_left);
        rainbow().applyTo(m_right);
        break;

      case DRIVE_FINKLE:
        rainbowFlash().applyTo(m_left);
        rainbowFlash().applyTo(m_right);
        break;

      case ESTOP:
        currentColor = new Color[] {Color.kGreen, Color.kGreenYellow};
        break;

      case OOOPS: // We tipped over
        currentColor = new Color[] {Color.kOrange};
        blink(currentColor).applyTo(m_left);
        blink(currentColor).applyTo(m_right);
        break;

        // IDLE & default case... just show alliance color
      case IDLE:
      default:
        currentColor = getAllianceColor();
        wave(currentColor).applyTo(m_left);
        wave(currentColor).applyTo(m_right);
        break;
    }
  }

  private static void updateCoralLEDs() {
    Color color = new Color(255, 63, 13);
    Color[] currentCoralColor = new Color[] {color, Color.kBlack};

    switch (Dashboard.getCoralState()) {
      case LAUNCHING:
        flash(currentCoralColor).applyTo(m_back_top);
        break;
      case RAMPING_UP:
        currentColor = new Color[] {color, Color.kBlack};
        breatheFast(currentCoralColor).applyTo(m_back_top);
        break;
      case IDLE:
        LEDPattern new_pattern = LEDPattern.solid(Color.kBlack);
        new_pattern.applyTo(m_back_top);
        break;
      case INTAKING:
        currentColor = new Color[] {color, Color.kBlack};
        breatheSlow(currentCoralColor).applyTo(m_back_top);
        break;
      case LOADED:
        new_pattern = LEDPattern.solid(color);
        new_pattern.applyTo(m_back_top);
        break;
      default:
        break;
    }
    switch (Commodore.getCurrentState()) {
      case OOOPS: // We tipped over
        currentColor = new Color[] {Color.kOrange};
        blink(currentColor).applyTo(m_back_top);
        break;
      default:
        break;
    }
  }

  private static void updateAlgaeLEDs() {
    Color color = new Color(25, 175, 25);
    Color[] currentAlgaeColor = new Color[] {color, Color.kBlack};

    switch (Dashboard.getAlgaeState()) {
      case LAUNCHING:
        flash(currentAlgaeColor).applyTo(m_back_bottom);
        break;
      case RAMPING_UP:
        breatheFast(currentAlgaeColor).applyTo(m_back_bottom);
        break;
      case IDLE:
        LEDPattern new_pattern = LEDPattern.solid(Color.kBlack);
        new_pattern.applyTo(m_back_bottom);
        break;
      case INTAKING:
        breatheSlow(currentAlgaeColor).applyTo(m_back_bottom);
        break;
      case LOADED:
        new_pattern = LEDPattern.solid(color);
        new_pattern.applyTo(m_back_bottom);
        break;
      default:
        break;
    }
    switch (Commodore.getCurrentState()) {
      case OOOPS: // We tipped over
        currentColor = new Color[] {Color.kOrange};
        blink(currentColor).applyTo(m_back_bottom);
        break;
      default:
        break;
    }
  }

  private static void updateStatusLEDs() {
    // rsl light
    boolean rsl = RobotController.getRSLState();
    int i = LEDConstants.kLEDSectionStatusLeft.startId();
    int j = LEDConstants.kLEDSectionStatusRight.endId();

    LEDBuffer.setRGB(i, rsl ? 255 : 0, rsl ? 40 : 0, 0);
    LEDBuffer.setRGB(i + 1, rsl ? 255 : 0, rsl ? 40 : 0, 0);
    LEDBuffer.setRGB(j, rsl ? 255 : 0, rsl ? 40 : 0, 0);
    LEDBuffer.setRGB(j - 1, rsl ? 255 : 0, rsl ? 40 : 0, 0);
    // brown out
    boolean brownOut = RobotController.isBrownedOut();
    LEDBuffer.setRGB(i + 2, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer.setRGB(i + 3, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer.setRGB(j - 2, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
    LEDBuffer.setRGB(j - 3, brownOut ? 255 : 0, brownOut ? 0 : 255, 0);
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
    LEDBuffer.setRGB(i + 4, r, g, b);
    LEDBuffer.setRGB(i + 5, r, g, b);
    LEDBuffer.setRGB(j - 4, r, g, b);
    LEDBuffer.setRGB(j - 5, r, g, b);
  }

  private static LEDPattern blink(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .blink(LEDConstants.blinkDuration);
    return new_pattern;
  }

  private static LEDPattern flash(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .blink(LEDConstants.flashDuration);
    return new_pattern;
  }

  private static LEDPattern breatheFast(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .breathe(LEDConstants.breathFastDuration);
    return new_pattern;
  }

  private static LEDPattern breatheSlow(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .breathe(LEDConstants.breathSlowDuration);
    return new_pattern;
  }

  private static LEDPattern wave(Color... colors) {
    LEDPattern new_pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
            .scrollAtAbsoluteSpeed(LEDConstants.scrollVelocity, LEDConstants.ledSpacing);
    return new_pattern;
  }

  private static LEDPattern rainbow() {
    LEDPattern rainbow =
        LEDPattern.rainbow(LEDConstants.rainbowSaturation, LEDConstants.rainbowValue)
            .scrollAtAbsoluteSpeed(LEDConstants.scrollVelocity, LEDConstants.ledSpacing);

    return rainbow;
  }

  private static LEDPattern rainbowFlash() {
    LEDPattern rainbow =
        LEDPattern.rainbow(LEDConstants.rainbowSaturation, LEDConstants.rainbowValue)
            .blink(LEDConstants.flashDuration);

    return rainbow;
  }

  private static LEDPattern progressrainbow() {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.2, Color.kBlack);

    LEDPattern base = LEDPattern.rainbow(LEDConstants.rainbowSaturation, LEDConstants.rainbowValue);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtAbsoluteSpeed(LEDConstants.scrollVelocity, LEDConstants.ledSpacing);
    return base.mask(mask);
  }

  private static Color[] getAllianceColor() {
    Color[] newColor = new Color[] {Color.kHotPink, Color.kPink};

    // Set the color based on the alliance
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        newColor = new Color[] {Color.kRed, Color.kBlack, Color.kBlack};
      }
      if (ally.get() == Alliance.Blue) {
        newColor = new Color[] {Color.kNavy, Color.kBlack, Color.kBlack};
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
    // updateStatusLEDs();

    LEDs.setData(LEDBuffer);
  }
}
