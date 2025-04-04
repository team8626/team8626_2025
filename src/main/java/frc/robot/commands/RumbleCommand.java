// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.utils.CS_XboxController;
import java.util.function.BooleanSupplier;

public class RumbleCommand extends CS_Command {

  GenericHID m_controller;
  Runnable m_execute;
  BooleanSupplier m_isFinished;
  private Timer m_timer = new Timer();
  private double m_intensity;
  private RumbleType m_rumbleType;
  private int m_currentPulseCount;
  // Change name
  private boolean m_isPulsing;
  private int m_pulseCount;

  public RumbleCommand() {
    this(RobotContainer.driverController, 1, 0.5, RumbleType.kBothRumble);
  }

  public RumbleCommand(CS_XboxController controller, RumbleType rumbleType) {
    this(controller, 1, 0.5, rumbleType);
  }

  public RumbleCommand(
      CS_XboxController controller, double intensity, double time, RumbleType rumbleType) {
    m_controller = controller;
    m_intensity = intensity;
    m_rumbleType = rumbleType;
    m_execute = () -> {};
    m_isFinished = () -> m_timer.hasElapsed(time);
  }

  public RumbleCommand(
      CS_XboxController controller,
      double intensity,
      int pulseCount,
      double pulseLength,
      double pulsePauseLength,
      RumbleType rumbleType) {
    m_rumbleType = rumbleType;
    m_controller = controller;
    m_intensity = intensity;
    m_isPulsing = true;
    m_pulseCount = pulseCount;
    m_execute =
        () -> {
          if (m_isPulsing) {
            if (m_timer.hasElapsed(pulseLength)) {
              m_controller.setRumble(m_rumbleType, 0);
              m_timer.reset();
              m_currentPulseCount++;
              m_isPulsing = false;
            }
          } else {
            if (m_timer.hasElapsed(pulsePauseLength)) {
              m_controller.setRumble(m_rumbleType, m_intensity);
              m_timer.reset();
              m_isPulsing = true;
            }
          }
        };

    m_isFinished = () -> arePulseCountCompleted();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_currentPulseCount = 0;
    m_controller.setRumble(m_rumbleType, m_intensity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_execute.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(m_rumbleType, 0);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished.getAsBoolean();
  }

  private boolean arePulseCountCompleted() {
    return m_currentPulseCount == m_pulseCount;
  }

  public static RumbleCommand longRumble(CS_XboxController controller) {
    return new RumbleCommand(controller, 1, 2, RumbleType.kBothRumble);
  }

  public static RumbleCommand longRumble() {
    return longRumble(RobotContainer.driverController);
  }
}
