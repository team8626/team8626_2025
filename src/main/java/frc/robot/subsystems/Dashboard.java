// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// This is a dashboard running a local webserver for operator interaction.
// the UI is located in deploy/public
//
// It is intended for use in the FIRST Robotics Competition.
//
// Original Work by Team 6328 Mechanical Adnvantage

package frc.robot.subsystems;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.RobotConstants.UIConstants.AlgaeFace2;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.RobotConstants.UIConstants.DTP;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.RobotContainer;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Dashboard extends CS_SubsystemBase {
  // Registration and hnadling of subsystems
  private static List<CS_SubsystemBase> subsystemsShortInterval = new ArrayList<>();
  private static List<CS_SubsystemBase> subsystemsLongInterval = new ArrayList<>();

  private static final boolean kEnableDashBoard = true;
  private static final double kShortInterval = .02; //  20ms
  private static final double kLongInterval = .5; // 500ms

  private double m_shortOldTime = 0.0;
  private double m_longOldTime = 0.0;

  public enum UpdateInterval {
    SHORT_INTERVAL,
    LONG_INTERVAL
  }

  // GamePieceState enum
  private static GamePieceState coralState = GamePieceState.IDLE;
  private static GamePieceState algaeState = GamePieceState.IDLE;

  public enum GamePieceState {
    RAMPING_UP,
    LAUNCHING,
    INTAKING,
    IDLE,
    LOADED
  }

  // Our own autochooser
  private SendableChooser<AutoOptions> mainAutoChooser = new SendableChooser<AutoOptions>();

  public enum AutoOptions {
    F_RIGHT_C,
    E1_RIGHT_C,
    H,
    H_LEFT_L,
    J_LEFT_K,
    G_RIGHT_C,
    DO_NOTHING,
    SOURCE,
    TRAJECTORY,
  };

  // NetworkTables Topics for communication
  private static final String toRobotTable = "/UIDashboard/ToRobot";
  private static final String toDashboardTable = "/UIDashboard/ToDashboard";
  private static final String allianceColorTopicName = "allianceColor";
  private static final String allowedCoralLevelsTopicName = "allowedCoralLevels";
  private static final String selectedCoralLevelTopicName = "selectedCoralLevel";
  private static final String selectedCoralBranchTopicName = "selectedCoralBranch";
  private static final String selectedAlgaeFaceTopicName = "selectedAlgaeFace";
  private static final String selectedPickupSideTopicName = "selectedPickupSide";
  private static final String selectedDtpTopicName = "selectedDtp";
  private static final String selectedMatchTimeTopicName = "matchTime";
  private static final String isAutoTimeTopicName = "isAuto";
  private static final String algaeStateTopicName = "algaeState";
  private static final String coralStateTopicName = "coralState";
  private static final String coralShootTimeTopicName = "coralLastShootTime";
  private static final String algaeShootTimeTopicName = "algaeLastShootTime";
  private static final String resetCoralBranchTopicName = "resetCoralBranch";
  private static final String resetAlgaeFaceTopicName = "resetAlgaeFace";

  private static final Timer timer = new Timer();

  // Values for the UIDashboard
  private static final ReefControlsValues inputs = new ReefControlsValues();

  static class ReefControlsValues {
    public int[] selectedLevel = new int[] {}; // 0 = L2, 1 = L3, 2 = L4
    public int[] selectedCoralLevel = new int[] {}; // Count
    public int[] selectedCoralBranch = new int[] {}; // Bitfield
    public int[] selectedAlgaeFace = new int[] {}; // Bitfield
    public int[] selectedPickupSide = new int[] {}; // Bitfield
    public boolean[] selectedDtp = new boolean[] {}; // Boolean
  }

  // Subscribers and Publishers
  private final IntegerSubscriber selectedCoralLevelIn;
  private final IntegerSubscriber selectedCoralBranchIn;
  private final IntegerSubscriber selectedAlgaeFaceIn;
  private final IntegerSubscriber selectedPickupSideIn;
  private final BooleanSubscriber selectedDtpIn;

  private final IntegerPublisher allianceColorOut;
  private final IntegerPublisher allowedCoralLevelsOut;
  private static IntegerPublisher selectedCoralLevelOut;
  private static IntegerPublisher selectedCoralBranchOut;
  private static IntegerPublisher selectedAlgaeFaceOut;
  private final IntegerPublisher selectedPickupSideOut;
  private final BooleanPublisher selectedDtpOut;
  private final DoublePublisher selectedMatchTimeOut;
  private final BooleanPublisher isAutoOut;
  private final StringPublisher algaeStateOut;
  private final StringPublisher coralStateOut;
  private static IntegerPublisher algaeShootTimeOut;
  private static IntegerPublisher coralShootTimeOut;
  private static BooleanPublisher resetCoralBranchOut;
  private static BooleanPublisher resetAlgaeFaceOut;

  public Dashboard() {
    // Create subscribers
    var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);
    selectedCoralLevelIn =
        inputTable
            .getIntegerTopic(selectedCoralLevelTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    selectedCoralBranchIn =
        inputTable
            .getIntegerTopic(selectedCoralBranchTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    selectedAlgaeFaceIn =
        inputTable
            .getIntegerTopic(selectedAlgaeFaceTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    selectedPickupSideIn =
        inputTable
            .getIntegerTopic(selectedPickupSideTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    selectedDtpIn =
        inputTable
            .getBooleanTopic(selectedDtpTopicName)
            .subscribe(false, PubSubOption.keepDuplicates(true));

    // Create publishers
    var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);
    allianceColorOut = outputTable.getIntegerTopic(allianceColorTopicName).publish();
    allowedCoralLevelsOut = outputTable.getIntegerTopic(allowedCoralLevelsTopicName).publish();
    selectedCoralLevelOut = outputTable.getIntegerTopic(selectedCoralLevelTopicName).publish();
    selectedCoralBranchOut = outputTable.getIntegerTopic(selectedCoralBranchTopicName).publish();
    selectedAlgaeFaceOut = outputTable.getIntegerTopic(selectedAlgaeFaceTopicName).publish();
    selectedDtpOut = outputTable.getBooleanTopic(selectedDtpTopicName).publish();
    selectedPickupSideOut = outputTable.getIntegerTopic(selectedPickupSideTopicName).publish();
    isAutoOut = outputTable.getBooleanTopic(isAutoTimeTopicName).publish();
    selectedMatchTimeOut = outputTable.getDoubleTopic(selectedMatchTimeTopicName).publish();
    algaeStateOut = outputTable.getStringTopic(algaeStateTopicName).publish();
    coralStateOut = outputTable.getStringTopic(coralStateTopicName).publish();
    algaeShootTimeOut = outputTable.getIntegerTopic(algaeShootTimeTopicName).publish();
    coralShootTimeOut = outputTable.getIntegerTopic(coralShootTimeTopicName).publish();
    resetCoralBranchOut = outputTable.getBooleanTopic(resetCoralBranchTopicName).publish();
    resetAlgaeFaceOut = outputTable.getBooleanTopic(resetAlgaeFaceTopicName).publish();

    // Start web server
    WebServer.start(
        5801,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "public")
            .toString());

    // Set default values
    this.setDefaultValues();

    // Create auto chooser drop down
    mainAutoChooser.addOption("DO NOTHING", AutoOptions.DO_NOTHING);
    mainAutoChooser.addOption("E1_RIGHT_C", AutoOptions.E1_RIGHT_C);
    mainAutoChooser.addOption("F_RIGHT_C", AutoOptions.F_RIGHT_C);
    mainAutoChooser.addOption("G_RIGHT_C", AutoOptions.G_RIGHT_C);
    mainAutoChooser.addOption("J_LEFT_K", AutoOptions.J_LEFT_K);
    mainAutoChooser.addOption("H", AutoOptions.H);
    mainAutoChooser.addOption("H_LEFT_L", AutoOptions.H_LEFT_L);
    mainAutoChooser.addOption("SOURCE", AutoOptions.SOURCE);

    mainAutoChooser.setDefaultOption("TRAJECTORY", AutoOptions.TRAJECTORY);

    mainAutoChooser.setDefaultOption("TRAJECTORY", AutoOptions.TRAJECTORY);

    SmartDashboard.putData("Autos/Autonomous Mode", mainAutoChooser);

    if (Robot.isSimulation()) {
      timer.start();
    }
  }

  /**
   * Registers a subsystem with the specified update interval.
   *
   * @param subsystem The subsystem to register.
   * @param interval The update interval for the subsystem (SHORT_INTERVAL or LONG_INTERVAL). By
   *     default (no parameter) the update interval is SHORT_INTERVAL
   */
  public static void registerSubsystem(CS_SubsystemBase subsystem, UpdateInterval interval) {
    if (interval == UpdateInterval.LONG_INTERVAL) {
      subsystemsLongInterval.add(subsystem);
    } else {
      subsystemsShortInterval.add(subsystem);
    }
    System.out.println(
        "[DASHBOARD] Registered "
            + subsystem.getClass().getName()
            + " for "
            + interval
            + " updates");
  }

  /**
   * Registers a subsystem with the default update interval (SHORT_INTERVAL).
   *
   * @param subsystem The subsystem to register.
   */
  public static void registerSubsystem(CS_SubsystemBase subsystem) {
    registerSubsystem(subsystem, UpdateInterval.SHORT_INTERVAL);
  }

  public static List<CS_SubsystemBase> listRegisteredSubsystems() {
    List<CS_SubsystemBase> combinedIntervals = new ArrayList<>();
    combinedIntervals.addAll(subsystemsShortInterval);
    combinedIntervals.addAll(subsystemsLongInterval);
    return combinedIntervals;
  }

  public void updateValues() {
    inputs.selectedCoralLevel =
        selectedCoralLevelIn.readQueue().length > 0
            ? new int[] {(int) selectedCoralLevelIn.get()}
            : new int[] {};

    inputs.selectedCoralBranch =
        selectedCoralBranchIn.readQueue().length > 0
            ? new int[] {(int) selectedCoralBranchIn.get()}
            : new int[] {};

    inputs.selectedAlgaeFace =
        selectedAlgaeFaceIn.readQueue().length > 0
            ? new int[] {(int) selectedAlgaeFaceIn.get()}
            : new int[] {};

    inputs.selectedPickupSide =
        selectedPickupSideIn.readQueue().length > 0
            ? new int[] {(int) selectedPickupSideIn.get()}
            : new int[] {};

    inputs.selectedDtp =
        selectedDtpIn.readQueue().length > 0
            ? new boolean[] {selectedDtpIn.get()}
            : new boolean[] {};
  }

  private void setAllianceColor(int value) {
    allianceColorOut.set(getAllianceColorAsInt());
  }

  private void setAllowedCoralLevels(List<CoralLevel> allowedcorallevels2) {
    // TODO: allowedCoralLevelsOut.set(allowedcorallevels2);
  }

  public static void setSelectedCoralLevel(int value) {
    selectedCoralLevelOut.set(value);
  }

  public static void setSelectedCoralBranch(int value) {
    selectedCoralBranchOut.set(value);
  }

  public static void setSelectedAlgaeFace(int value) {
    selectedAlgaeFaceOut.set(value);
  }

  private void setSelectedPickupSide(int value) {
    selectedPickupSideOut.set(value);
  }

  private void setSelectedDtp(boolean value) {
    selectedDtpOut.set(value);
  }

  private void setMatchTime(double value) {
    selectedMatchTimeOut.set(value);
  }

  public static void setResetCoralBranch(boolean value) {
    resetCoralBranchOut.set(value);
  }

  public static void setResetAlgaeFace(boolean value) {
    resetAlgaeFaceOut.set(value);
  }

  private int getAllianceColorAsInt() {
    int retVal = 0;

    if (DriverStation.isDSAttached() && DriverStation.isFMSAttached()) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          retVal = 1; // Alliance is red
        } else if (ally.get() == Alliance.Blue) {
          retVal = 2; // Alliance is blue
        }
      }
    }
    return retVal;
  }

  private void setDefaultValues() {
    setAllowedCoralLevels(UIConstants.allowedCoralLevels2);
    setSelectedCoralLevel(UIConstants.defaultCoralLevel.getValue());
    setSelectedCoralBranch(UIConstants.defaultCoralBranch.getValue());
    setSelectedAlgaeFace(UIConstants.defaultAlgaeFace.getValue());
    setSelectedPickupSide(UIConstants.defaultPickupSide.getValue());
    setSelectedDtp(UIConstants.defaultDTP.getValue());
    setResetCoralBranch(false);
    setResetAlgaeFace(false);
    setMatchTime(0);
  }

  public static CoralLevel getSelectedCoralLevel() {
    return inputs.selectedCoralLevel.length > 0
        ? CoralLevel.getByValue(inputs.selectedCoralLevel[0])
        : null;
  }

  public static CoralBranch getSelectedCoralBranch() {
    return inputs.selectedCoralBranch.length > 0
        ? CoralBranch.getByValue(inputs.selectedCoralBranch[0])
        : null;
  }

  public static AlgaeFace2 getSelectedAlgaeFace() {
    return inputs.selectedAlgaeFace.length > 0
        ? AlgaeFace2.getByValue(inputs.selectedAlgaeFace[0])
        : null;
  }

  public static PickupSide getSelectedPickupSide() {
    return inputs.selectedPickupSide.length > 0
        ? PickupSide.getByValue(inputs.selectedPickupSide[0])
        : null;
  }

  public static DTP getSelectedDtp() {
    return inputs.selectedDtp.length > 0 ? DTP.getByValue(inputs.selectedDtp[0]) : null;
  }

  @Override
  public void CS_periodic() {
    // Call the update method for all registered subsystems
    double time = Timer.getFPGATimestamp();
    if (kEnableDashBoard) {
      // Update short interval
      if ((time - m_shortOldTime) > kShortInterval) {
        m_shortOldTime = time;
        for (CS_SubsystemBase s : subsystemsShortInterval) {
          s.updateDashboard();
        }
      }
      // Update long interval
      if ((time - m_longOldTime) > kLongInterval) {
        // Thing that should be updated every LONG_DELAY
        m_longOldTime = time;
        for (CS_SubsystemBase s : subsystemsShortInterval) {
          s.updateDashboard();
        }
        for (CS_SubsystemBase s : subsystemsLongInterval) {
          s.updateDashboard();
        }
      }
    }

    // Get Subsystem Steady States
    // (Could be a preloaded piece or a reboot...)
    if (coralState == GamePieceState.IDLE || coralState == GamePieceState.LOADED) {
      if (RobotContainer.mortar != null && RobotContainer.mortar.isLoaded()) {
        coralState = GamePieceState.LOADED;
      } else {
        coralState = GamePieceState.IDLE;
      }
    }
    if (algaeState == GamePieceState.IDLE || algaeState == GamePieceState.LOADED) {
      if (RobotContainer.algae501 != null && RobotContainer.algae501.isLoaded()) {
        algaeState = GamePieceState.LOADED;
      } else {
        algaeState = GamePieceState.IDLE;
      }
    }

    // Read values from NetworkTables
    this.updateValues();

    // Update values in NetworkTables
    this.setAllianceColor(0);
    if (Robot.isSimulation()) {
      this.selectedMatchTimeOut.set(timer.get());
    } else {
      this.selectedMatchTimeOut.set(DriverStation.getMatchTime());
    }
    this.isAutoOut.set(DriverStation.isAutonomous());
    this.algaeStateOut.set(Dashboard.getAlgaeState().toString());
    this.coralStateOut.set(Dashboard.getCoralState().toString());
  }

  public static void publishCoralShootTime(int timeMs) {
    coralShootTimeOut.set(timeMs);
  }

  public static void publishAlgaeShootTime(int timeMs) {
    algaeShootTimeOut.set(timeMs);
  }

  public static void setCoralState(GamePieceState new_state) {
    coralState = new_state;
  }

  public static GamePieceState getCoralState() {
    return coralState;
  }

  public static GamePieceState getAlgaeState() {
    return algaeState;
  }

  public static void setAlgaeState(GamePieceState new_state) {
    algaeState = new_state;
  }

  public static Command getSetCoralStateCommand(GamePieceState new_state) {
    return new InstantCommand(() -> Dashboard.setCoralState(new_state));
  }

  public static Command getSetAlgaeStateCommand(GamePieceState new_state) {
    return new InstantCommand(() -> Dashboard.setAlgaeState(new_state));
  }

  public AutoOptions getSelectedAuto() {
    return mainAutoChooser.getSelected();
  }

  // public static void resetCoralBranch() {
  //   resetCoralBranchOut.set(true);
  // }

  // public static void resetAlgaeFace() {
  //   resetAlgaeFaceOut.set(true);
  // }

  // public static void resetCoralBranch(boolean value) {
  //   resetCoralBranchOut.set(value);
  // }

  // public static void resetAlgaeFace(boolean value) {
  //   resetAlgaeFaceOut.set(value);
  // }
}
