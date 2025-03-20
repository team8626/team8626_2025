// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// This state machine is inspired by the robot code developed by Team 1768 Nashoba Robotics.
// It is intended for use in the FIRST Robotics Competition.
//
// Original Work by Team 1768 Nashoba Robotics

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.setters.groups.*;
import frc.robot.subsystems.CS_SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class Commodore extends CS_SubsystemBase {
  // Singleton instance
  private static Commodore instance;

  private static CommodoreState currentState = CommodoreState.UNKNOWN;
  private static Queue<CommodoreState> stateHistory;

  public enum CommodoreState {
    BOOT,
    ESTOP,
    ERROR_CRITICAL,
    DISCONNECTED,
    DISABLED,
    IDLE,

    UNKNOWN,
    TRANSITION,

    // CORAL_SHOOT,
    // CORAL_SHOOT_RAMPINGUP,
    // CORAL_SHOOT_LAUNCHING,
    // CORAL_INTAKE,
    // CORAL_LOADED,

    // ALGAE_SHOOT_SETTINGSUBSYSTEMS,
    // ALGAE_SHOOT,
    // ALGAE_SHOOT_RAMPINGUP,
    // ALGAE_SHOOT_LAUNCHING,
    // ALGAE_INTAKE,
    // ALGAE_LOADED,

    DRIVE_AUTO,
    DRIVE_FINKLE,

    SUBSYSTEMS_ADJUST,
    SUBSYSTEMS_AT_SETPOINT,

    ELEVATOR_ZEROING,

    TUNE_CORALSHOOTER,
    TUNE_ALGAESHOOTER,

    CLIMB_PREP,
    CLIMB_READY,
    CLIMB_NOW,
    GO_TO_POSITION
  }

  // Private Constructor to prevent instantiation
  private Commodore() {
    super();
    currentState = CommodoreState.UNKNOWN;

    // Initialize the state history queue
    stateHistory = new LinkedList<>();
    stateHistory.add(CommodoreState.UNKNOWN);
    stateHistory.add(CommodoreState.UNKNOWN);
    stateHistory.add(CommodoreState.UNKNOWN);
    stateHistory.add(CommodoreState.UNKNOWN);
    stateHistory.add(CommodoreState.UNKNOWN);
  }

  // Public method to provide access to the singleton instance
  public static Commodore getInstance() {
    if (instance == null) {
      instance = new Commodore();
    }
    return instance;
  }

  /**
   * Set the robot state
   *
   * @param newState The state to set the robot to
   */
  public static Commodore setCommodoreState(CommodoreState newState) {
    applyState(newState);
    return getInstance();
  }

  private static void applyState(CommodoreState newState) {
    if (newState != currentState) {
      pushLastState(currentState);
      currentState = newState;

      getInstance().printf("New State: %s\n", newState.toString());
    }
    // Same state, nothing to do
    else {
    }
  }

  public static CommodoreState getCurrentState() {
    return currentState;
  }

  public static CommodoreState getLastState() {
    return getLastState(0);
  }

  // Method to add a state to the last states queue
  private static void pushLastState(CommodoreState state) {
    if (stateHistory.size() == 5) {
      stateHistory.poll(); // Remove the oldest state
    }
    stateHistory.add(state);
  }

  // Method to get the n-th element of the last states queue
  public static CommodoreState getLastState(int n) {
    CommodoreState retVal = CommodoreState.UNKNOWN;
    if (stateHistory != null) {
      if (n < 0 || n >= stateHistory.size()) {
        throw new IndexOutOfBoundsException("Index: " + n + ", Size: " + stateHistory.size());
      }
      List<CommodoreState> list = new LinkedList<>(stateHistory);
      retVal = list.get(stateHistory.size() - n - 1);
    }
    return retVal;
  }

  public static Command getSetStateCommand(CommodoreState state) {
    SmartDashboard.putString("Commodore/Desired State", state.toString());

    return new InstantCommand(() -> Commodore.applyState(state));
  }

  @Override
  public void initDashboard() {
    updateDashboard();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Commodore/Last State", getLastState(0).toString());
    SmartDashboard.putString("Commodore/Current State", Commodore.currentState.toString());
  }

  @Override
  public void CS_periodic() {
    // Check for E-Stop
    if (DriverStation.isEStopped()) {
      setCommodoreState(CommodoreState.ESTOP);
    }
  }
}
