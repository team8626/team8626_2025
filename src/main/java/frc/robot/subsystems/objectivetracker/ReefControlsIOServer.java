// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.objectivetracker;

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
import frc.robot.Robot;
import frc.robot.RobotConstants.UIConstants;
import frc.robot.RobotConstants.UIConstants.AlgaeFace2;
import frc.robot.RobotConstants.UIConstants.CoralBranch;
import frc.robot.RobotConstants.UIConstants.CoralLevel;
import frc.robot.RobotConstants.UIConstants.DTP;
import frc.robot.RobotConstants.UIConstants.PickupSide;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.Dashboard;
import java.nio.file.Paths;
import java.util.Optional;

public class ReefControlsIOServer extends CS_SubsystemBase {
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

  private static final Timer timer = new Timer();

  static class ReefControlsValues {
    public int[] selectedLevel = new int[] {}; // 0 = L2, 1 = L3, 2 = L4
    public int[] selectedCoralLevel = new int[] {}; // Count
    public int[] selectedCoralBranch = new int[] {}; // Bitfield
    public int[] selectedAlgaeFace = new int[] {}; // Bitfield
    public int[] selectedPickupSide = new int[] {}; // Bitfield
    public boolean[] selectedDtp = new boolean[] {}; // Boolean
  }

  // private final IntegerSubscriber allianceColorIn;
  // private final IntegerSubscriber allowedCoralLevelsIn;
  private final IntegerSubscriber selectedCoralLevelIn;
  private final IntegerSubscriber selectedCoralBranchIn;
  private final IntegerSubscriber selectedAlgaeFaceIn;
  private final IntegerSubscriber selectedPickupSideIn;
  private final BooleanSubscriber selectedDtpIn;

  private final IntegerPublisher allianceColorOut;
  private final IntegerPublisher allowedCoralLevelsOut;
  private final IntegerPublisher selectedCoralLevelOut;
  private final IntegerPublisher selectedAlgaeFaceOut;
  // private final IntegerPublisher selectedPickupSideOut;
  private final BooleanPublisher selectedDtpOut;
  private final DoublePublisher selectedMatchTimeOut;
  private final BooleanPublisher isAutoOut;
  private final StringPublisher algaeStateOut;
  private final StringPublisher coralStateOut;

  private static final ReefControlsValues inputs = new ReefControlsValues();

  public ReefControlsIOServer() {
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
    selectedAlgaeFaceOut = outputTable.getIntegerTopic(selectedAlgaeFaceTopicName).publish();
    // selectedPickupSideOut = outputTable.getIntegerTopic(selectedPickupSideTopicName).publish();
    selectedDtpOut = outputTable.getBooleanTopic(selectedDtpTopicName).publish();
    isAutoOut = outputTable.getBooleanTopic(isAutoTimeTopicName).publish();
    selectedMatchTimeOut = outputTable.getDoubleTopic(selectedMatchTimeTopicName).publish();
    algaeStateOut = outputTable.getStringTopic(algaeStateTopicName).publish();
    coralStateOut = outputTable.getStringTopic(coralStateTopicName).publish();

    // Start web server
    WebServer.start(
        5801,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols")
            .toString());

    // Set default values
    this.setDefaultValues();

    if (Robot.isSimulation()) {
      timer.start();
    }
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

  private void setAllowedCoralLevels(int value) {
    allowedCoralLevelsOut.set(value);
  }

  private void setSelectedCoralLevel(int value) {
    selectedCoralLevelOut.set(value);
  }

  private void setSelectedAlgaeFace(int value) {
    selectedAlgaeFaceOut.set(value);
  }

  private void setSelectedPickupSide(int value) {
    // selectedPickupSideOut.set(value);
  }

  private void setSelectedDtp(boolean value) {
    selectedDtpOut.set(value);
  }

  private void setSelectedMatchTime(double value) {
    selectedMatchTimeOut.set(value);
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
    this.setAllowedCoralLevels(0);
    this.setSelectedCoralLevel(UIConstants.defaultCoralLevel.getValue());
    this.setSelectedAlgaeFace(0);
    this.setSelectedPickupSide(UIConstants.defaultPickupSide.getValue());
    this.setSelectedDtp(UIConstants.defaultDTP.getValue());
    this.setSelectedMatchTime(0);
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
}
