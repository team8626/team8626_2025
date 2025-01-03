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

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.setters.ToIdle;
import frc.robot.commands.setters.ToIntake;
import frc.robot.commands.setters.ToShoot;
// import frc.robot.commands.setters.groups...;
import frc.robot.subsystems.CS_SubsystemBase;

public class Commodore extends CS_SubsystemBase{
    // Singleton instance
    private static Commodore instance;

    private static CommodoreState currentState = CommodoreState.UNKNOWN;
    private static Queue<CommodoreState> stateHistory;
    
    private static Boolean isToggleState = false;

    public enum CommodoreState {
        BOOT,
        ESTOP,
        ERROR_CRITICAL,
        DISCONNECTED,
        DISABLED,
        IDLE,

        UNKNOWN,
        TRANSITION,

        INTAKE,
        SHOOT
    }

    // Private Constructor to prevent instantiation
    private Commodore(){
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
     * @param newState The state to set the robot to
     */
    public Commodore setCommodoreState(CommodoreState newState) {
        return setCommodoreState(newState, false);
    }

    /**
     * Set the robot state
     * @param newState The state to set the robot to
     * @param override Whether to override the current state
     */
    public static Commodore setCommodoreState(CommodoreState newState, boolean override) {
    
        getInstance().printf("Setting State: Requested: %s%s, Current %s\n", newState.toString(), override?"(OVERRIDE)":"", currentState.toString());

        // If we are in the same state and it's a toggle, go back to IDLE
        if((currentState == newState) && (isToggleState == true)){
            newState = CommodoreState.IDLE;
            override = true;
        }

        // Reset the toggle state
        isToggleState = false;
   
        // If we are in a NOT in a TRANSITION state
        // OR we are overriding the current state
        //
        // ==> Launch the appropriate command (through local methods)
        //     the currentState will be updated by the command using the applyState() method
        //     (which will also update the stateHistory queue)
        if(override || currentState != CommodoreState.TRANSITION) {
            switch (newState) {
                 // This is the resting state, robot is enabled, but not doing anything
                 case IDLE:
                    toIdle();
                 break;

                 // Those cases are just for LED update
                 // The Robot is disabled in those states, there will be no command launched.
                case BOOT:
                case ESTOP:
                case DISCONNECTED:
                case DISABLED:
                case ERROR_CRITICAL:
                case UNKNOWN:
                    applyState(newState);
                    break;

                // Those cases are for launching commands
                // State will go to TRANSITION and then the command will update the state
                case INTAKE:
                    toIntake();
                    break; 
                case SHOOT:
                    toShoot();
                    break;

                // We shouldn't be there!
                case TRANSITION:
                    getInstance().println("########## Check your code, TRANSITION state should be overwritten by commands ##########");
                    break;
                default:
                    getInstance().printf("########## Check your code, %s not handled ##########\n", newState.toString());
                    break;
            }
        }
        return instance;
    }

    public Commodore withToggleState() {
        if(currentState == CommodoreState.TRANSITION){
            isToggleState = false;
            // getInstance().println("------ TRANSITION, No Toggle");
        } else {
            isToggleState = true;
            getInstance().println("------ ("+ currentState.toString() +") IT'S A TOGGLE");
        }
        return this;
    }
    
    private static void applyState(CommodoreState newState){
        if(newState != currentState){
            pushLastState(currentState);
            currentState = newState;
            getInstance().printf("New State to %s %s(was %s)\n", newState.toString(), isToggleState==true?"--TOGGLE ":"", getLastState().toString());
            // displayStateHistory();
        }
        // Same state, nothing to do
        else {
            getInstance().printf("Same state (%s) -- Do Nothing\n", newState.toString());
        }
    }
    public static CommodoreState getCurrentState() {
        return currentState;
    }

    public static CommodoreState getLastState()  {
        return getLastState(0);
    }

    // Method to add a state to the last states queue
    private static void pushLastState(CommodoreState state) {
        if (stateHistory.size() == 5) {
            stateHistory.poll(); // Remove the oldest state
        }
        stateHistory.add(state);
    }

    // Method to display the content of the last states queue
    private static void displayStateHistory() {
        getInstance().printf("Last States:");
        for (CommodoreState state : stateHistory) {
            getInstance().printf("  - %s",state);
        }
        getInstance().println("");
    }

    // Method to get the n-th element of the last states queue
    public static CommodoreState getLastState(int n) {
        CommodoreState retVal = CommodoreState.UNKNOWN;
        if(stateHistory != null) {
            if (n < 0 || n >= stateHistory.size()) {
            throw new IndexOutOfBoundsException("Index: " + n + ", Size: " + stateHistory.size());
            }
            List<CommodoreState> list = new LinkedList<>(stateHistory);
            retVal = list.get(stateHistory.size() - n -1);
        }
        return retVal;
    }

    private static void toIdle() {
        applyState(CommodoreState.TRANSITION);
        CommandScheduler.getInstance().schedule(new ToIdle());
    }
    private static void toIntake() {
        applyState(CommodoreState.TRANSITION);
        CommandScheduler.getInstance().schedule(new ToIntake());
    }
    private static void toShoot() {
        applyState(CommodoreState.TRANSITION);
        CommandScheduler.getInstance().schedule(new ToShoot());
    }

    public static Command getSetStateCommand(CommodoreState state) {
        SmartDashboard.putString("Commodore/Desired State", "Setting to: "  + state.toString());

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
            setCommodoreState(CommodoreState.ESTOP, true);
          }
    }
}