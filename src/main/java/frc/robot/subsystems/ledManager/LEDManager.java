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
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.subsystems.CS_SubsystemBase;
import java.util.Optional;

public class LEDManager extends CS_SubsystemBase {

  // Singleton instance
  private static LEDManager instance;

  /** Creates a new LEDSubsystem. */
  private static AddressableLED LEDs;

  private static AddressableLEDBuffer buffer;
  // private static AddressableLEDBufferView mainLEDs;

  private static AddressableLEDBufferView leftLEDs;
  private static AddressableLEDBufferView rightLEDs;
  private static AddressableLEDBufferView frontLEDs;
  private static AddressableLEDBufferView backLEDs;

  private static AddressableLEDBufferView ambianceLEDs;

  private static LEDPattern generalPattern;
  private static LEDPattern driverInfoPattern;
  private static LEDPattern ambiencePattern;

  // static LedMode mainMode = LedMode.OFF;
  static CommodoreState mainMode = CommodoreState.IDLE;

  static Color currentColor[] = {Color.kHotPink, Color.kPink};

  private LEDManager() {
    super();

    LEDs = new AddressableLED(LEDConstants.kLEDPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);
    LEDs.setLength(LEDConstants.kLEDStripLength);
    LEDs.setData(buffer);
    LEDs.start();

    // Create the differnt sections of the LEDs (views)
    leftLEDs =
        buffer.createView(LEDConstants.kSectionLeft.startId(), LEDConstants.kSectionLeft.endId());
    rightLEDs =
        buffer
            .createView(LEDConstants.kSectionRight.startId(), LEDConstants.kSectionRight.endId())
            .reversed();
    frontLEDs =
        buffer.createView(LEDConstants.kSectionFront.startId(), LEDConstants.kSectionFront.endId());
    backLEDs =
        buffer.createView(LEDConstants.kSectionBack.startId(), LEDConstants.kSectionBack.endId());
    ambianceLEDs =
        buffer.createView(
            LEDConstants.kSectionAmbiance.startId(), LEDConstants.kSectionAmbiance.endId());

    // Set the default patterns
    generalPattern = off();
    driverInfoPattern = off();
    ambiencePattern = off();
  }

  // Public method to provide access to the singleton instance
  public static LEDManager getInstance() {
    if (instance == null) {
      instance = new LEDManager();
    }
    return instance;
  }

  /** Update the main LED sections based on current set mode */
  private void updateMainLeds() {
    mainMode = Commodore.getCurrentState();

    switch (mainMode) {
      case DISCONNECTED:
        generalPattern = wave(Color.kHotPink, Color.kPink);
        break;

      case DISABLED:
        currentColor = getAllianceColor();

        generalPattern = blinkSlow(currentColor[0]);
        break;

      case IDLE:
        currentColor = getAllianceColor();
        generalPattern = wave(currentColor[0], currentColor[1]);
        driverInfoPattern = off();
        break;

      case ESTOP:
        // currentColor = new Color[] {Color.kGreen, Color.kGreenYellow};
        // breath(
        //     LEDConstants.kSectionMain,
        //     currentColor[0],
        //     currentColor[1],
        //     LEDConstants.breathDuration,
        //     Timer.getFPGATimestamp());
        break;

      case UNKNOWN:
      case TRANSITION:
      case SHOOT:
        // wave(LEDConstants.kSectionMain, Color.kGreen, Color.kBlack, 25, 2.0);
        break;
      case CORAL_SHOOT:
      case CORAL_SHOOT_RAMPINGUP:
      case CORAL_SHOOT_LAUNCHING:
        driverInfoPattern = blinkFast(Color.kCoral);
        break;

      case CORAL_INTAKE:
        driverInfoPattern = blinkSlow(Color.kCoral);
        break;

      case CORAL_LOADED:
        driverInfoPattern = solid(Color.kCoral);
        break;

      case INTAKE:
        // wave(LEDConstants.kSectionMain, Color.kOrange, Color.kBlack, 25, 2.0);
        break;

      case TUNE_CORALSHOOTER:
        // blink(LEDConstants.kSectionMain, Color.kOrange, 0.5);
        break;

      case ERROR_CRITICAL:
        // pulse(LEDConstants.kSectionMain, Color.kYellowGreen, 0.5, 2.0);
        break;

      default:
        // wave(LEDConstants.kSectionMain, currentColor[0], currentColor[1], 25, 2.0);
        break;
    }
  }

  private static LEDPattern off() {
    return solid(Color.kBlack);
  }

  private static LEDPattern solid(Color color) {
    LEDPattern new_pattern;
    new_pattern = LEDPattern.solid(color);
    return new_pattern;
  }

  private static LEDPattern wave(Color... colors) {
    LEDPattern new_pattern;

    new_pattern =
        LEDPattern.gradient(GradientType.kContinuous, colors)
            .scrollAtAbsoluteSpeed(LEDConstants.kLEDScrollSpeed, LEDConstants.kLEDSpacing);

    return new_pattern;
  }

  private static LEDPattern blinkFast(Color color) {
    LEDPattern new_pattern;

    new_pattern = LEDPattern.solid(color).breathe(LEDConstants.kBlinkFastPeriod);

    return new_pattern;
  }

  private static LEDPattern blinkSlow(Color color) {
    LEDPattern new_pattern;

    new_pattern = LEDPattern.solid(color).breathe(LEDConstants.kBlinkSlowPeriod);

    return new_pattern;
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

    generalPattern.applyTo(leftLEDs);
    generalPattern.applyTo(rightLEDs);
    driverInfoPattern.applyTo(frontLEDs);
    driverInfoPattern.applyTo(backLEDs);
    ambiencePattern.applyTo(ambianceLEDs);

    LEDs.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }
}
