package frc.robot;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.commands.setters.groups...;
import frc.robot.subsystems.CS_SubsystemBase;

public class Commodore extends CS_SubsystemBase{
    // Singleton instance
    private static Commodore instance;

    private static CommodoreState currentState = CommodoreState.UNKNOWN;
    private static CommodoreState queuedState  = CommodoreState.UNKNOWN;
    private static CommodoreState desiredState = CommodoreState.UNKNOWN;
    private static Queue<CommodoreState> lastStates;
    
    public enum CommodoreState {
        BOOT,
        ESTOP,
        DISCONNECTED,
        DISABLED,
        IDLE,

        UNKNOWN,
        TRANSITION,

        INTAKE,
        SHOOT_PREP,
        SHOOT,
        
        ERROR_CRITICAL
    }

    // Private Constructor to prevent instantiation
    private Commodore(){
        super();
        lastStates = new LinkedList<>();
        lastStates.add(CommodoreState.UNKNOWN);
        lastStates.add(CommodoreState.UNKNOWN);
        lastStates.add(CommodoreState.UNKNOWN);
        lastStates.add(CommodoreState.UNKNOWN);
        lastStates.add(CommodoreState.UNKNOWN);
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
    void setCommodoreState(CommodoreState newState) {
        setCommodoreState(newState, false);
    }

    /**
     * Set the robot state
     * @param newState The state to set the robot to
     * @param override Whether to override the current state
     */
    public static void setCommodoreState(CommodoreState newState, boolean override) {
        desiredState = newState;
        // lastState = currentState;
        // pushLastState(currentState);

        // If we are in a TRANSITION state, queue the desired state
        if(currentState == CommodoreState.TRANSITION && !override){
            queuedState = newState;
        }

        // If we are in a UNKNOWN state, force override the current state
        if(newState == CommodoreState.UNKNOWN){
            override = true;
        }

        // If we are in a NOT in a TRANSITION state
        // OR we are overriding the current state
        //
        // ==> Launch the appropriate command (through local methods)
        //     the currentState will be updated by the command using the applyState() method
        //     (which will also update the lastStates queue)
        if(override || currentState != CommodoreState.TRANSITION) {
            switch (newState) {
                // Those cases are just for LED update
                case BOOT:
                case ESTOP:
                case DISCONNECTED:
                case DISABLED:
                case IDLE:
                case UNKNOWN:
                    applyState(newState);
                    break;


                // Those cases are for launching commands
                case INTAKE:
                    toIntake();
                    break; 
                case SHOOT_PREP:
                    toShootPrep();
                    break;
                case SHOOT:
                    toShoot();
                    break;
                // We shouldn't be there!
                case TRANSITION:
                    System.out.println("[COMMODORE] Check your code, TRANSITION state should be overwritten by commands.");
                    break;
                default:
                    System.out.printf("[COMMODORE] ########## Check your code, %s not handled ##########\n", newState.toString());
                    break;
            }
        }
    }

    private static void applyState(CommodoreState newState){
        // TODO: Need to handle TRANSITION state?

        if(newState != currentState){
            pushLastState(currentState);
            currentState = newState;
            desiredState = newState;
            System.out.printf("[COMMODORE] New State to %s (was %s)\n", newState.toString(), getLastState().toString());
            displayLastStates();
        }
    }
    public static CommodoreState getCurrentState() {
        return currentState;
    }
    public static CommodoreState getDesiredState(){
        return desiredState;
    }
    public static CommodoreState getLastState()  {
        return getLastState(0);
    }

    public static CommodoreState getQueuedState() {
        return queuedState;
    }
    public static void setQueuedState(CommodoreState queuedState) {
        Commodore.queuedState = queuedState;
    }
    // Method to add a state to the last states queue
    private static void pushLastState(CommodoreState state) {
        if (lastStates.size() == 5) {
            lastStates.poll(); // Remove the oldest state
        }
        lastStates.add(state);
        System.out.printf("[COMMODORE] Pushed %s\n", state.toString());

    }

    // Method to display the content of the last states queue
    private static void displayLastStates() {
        System.out.print("Last States:");
        for (CommodoreState state : lastStates) {
            System.out.printf("  - %s",state);
        }
        System.out.println();
    }

    // Method to get the n-th element of the last states queue
    public static CommodoreState getLastState(int n) {
        CommodoreState retVal = CommodoreState.UNKNOWN;
        if(lastStates != null) {
            if (n < 0 || n >= lastStates.size()) {
            throw new IndexOutOfBoundsException("Index: " + n + ", Size: " + lastStates.size());
            }
            List<CommodoreState> list = new LinkedList<>(lastStates);
            retVal = list.get(lastStates.size() - n -1);
        }
        return retVal;
    }

    private static void toIdle() {
        applyState(CommodoreState.TRANSITION);
        // CommandScheduler.getInstance().schedule(new ToIdle());
    }
    private static void toIntake() {
        applyState(CommodoreState.TRANSITION);
        // CommandScheduler.getInstance().schedule(new ToIntake());
    }
    private static void toShootPrep() {
        applyState(CommodoreState.TRANSITION);
        // CommandScheduler.getInstance().schedule(new ToShootPrep());
    }
    private static void toShoot() {
        applyState(CommodoreState.TRANSITION);
        // CommandScheduler.getInstance().schedule(new ToShoot());
    }

    public static Command getSetStateCommand(CommodoreState state) {
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
        SmartDashboard.putString("Commodore/Desired State", Commodore.desiredState.toString());
        SmartDashboard.putString("Commodore/Queued State", Commodore.queuedState.toString());
    }

    @Override
    public void CS_periodic() {
        // Check for E-Stop
        if (DriverStation.isEStopped()) {
            setCommodoreState(CommodoreState.ESTOP, true);
          }
    }
}