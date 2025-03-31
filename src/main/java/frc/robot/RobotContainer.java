// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants.RobotType;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.setters.autos.Auto_4;
import frc.robot.commands.setters.autos.Auto_5;
import frc.robot.commands.setters.autos.Auto_A;
import frc.robot.commands.setters.autos.Auto_B;
import frc.robot.commands.setters.autos.Auto_C;
import frc.robot.commands.setters.autos.Auto_D;
import frc.robot.commands.setters.autos.Auto_E;
import frc.robot.commands.setters.autos.Auto_F;
import frc.robot.commands.setters.autos.Auto_G;
import frc.robot.commands.setters.autos.Auto_H;
import frc.robot.commands.setters.autos.Auto_I;
import frc.robot.commands.setters.autos.Auto_J;
import frc.robot.commands.setters.autos.Auto_K;
import frc.robot.commands.setters.autos.Auto_L;
import frc.robot.commands.setters.groups.ToAlgaeShoot;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndAlgaeIntake;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndAlgaeShoot;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndCoralIntake;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndCoralShoot;
import frc.robot.commands.setters.groups.ToSubsystemsPreset;
import frc.robot.commands.setters.units.AlgaeShooterDiscard;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.CoralShooterIntake;
import frc.robot.commands.setters.units.DriveTurnToAngle;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.AutoOptions;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_Sim;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_SparkFlex;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooter_Sim;
import frc.robot.subsystems.coralshooter.CoralShooter_SparkMax;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.Elevator_LinearSparkMax;
import frc.robot.subsystems.elevator.Elevator_Sim;
import frc.robot.subsystems.ledManager.LEDManager;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.Wrist_Sim;
import frc.robot.subsystems.wrist.Wrist_SparkFlex;
import frc.robot.vizualization.Visualization;
import frc.utils.CS_ButtonBoxController;
import frc.utils.CS_XboxController;
import java.io.File;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;

public class RobotContainer {
  // Singleton instance
  private static RobotContainer instance;

  // Instantiate the Dashboard
  private final Dashboard dashboard = new Dashboard();

  // Instantiate the Commodore running the robot state machine.
  private final Commodore commodore = Commodore.getInstance();

  // Instantiate the LED manager
  private final LEDManager ledManager = LEDManager.getInstance();

  // Instantiate the Preset manager
  private final PresetManager presetManager = PresetManager.getInstance();

  // Vizualizaiton (Only in Simulation)
  private Visualization visualization = Visualization.getInstance();

  //
  // ****************************************************************************************

  // Define subsystems and commands here
  // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
  public static SwerveSubsystem drivebase = null;
  public static ElevatorSubsystem elevator = null;
  public static WristSubsystem wrist = null;
  public static CoralShooterSubsystem mortar = null;
  public static AlgaeShooterSubsystem algae501 = null;

  // Controllers
  public static final CS_XboxController driverController =
      new CS_XboxController(OperatorConstants.DRIVER_CONTOLLER_PORT);
  private static final CS_XboxController operatorController =
      new CS_XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private static final CS_ButtonBoxController buttonBox =
      new CS_ButtonBoxController(OperatorConstants.BUTTON_BOX_PORT);

  private SendableChooser<Command> autoChooser;

  private RobotContainer() {
    // Display build information
    displayCredits();

    // Define Robot Subsystems
    if (Robot.isSimulation()) {
      RobotConstants.robotType = RobotConstants.RobotType.SIMBOT;
    }

    switch (RobotConstants.robotType) {
      case DART:
        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_dart"));
        break;
      case SIMBOT:
        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_devbot"));

        elevator = new ElevatorSubsystem(new Elevator_Sim());
        wrist = new WristSubsystem(new Wrist_Sim());
        algae501 = new AlgaeShooterSubsystem(new AlgaeShooter_Sim());
        mortar = new CoralShooterSubsystem(new CoralShooter_Sim());

        break;
      case COMPBOT:
      default:
        drivebase =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_tsunami"));
        elevator = new ElevatorSubsystem(new Elevator_LinearSparkMax());
        wrist = new WristSubsystem(new Wrist_SparkFlex());
        mortar = new CoralShooterSubsystem(new CoralShooter_SparkMax());
        algae501 = new AlgaeShooterSubsystem(new AlgaeShooter_SparkFlex());

        break;
    }

    // Configure the button bindings
    configureDriverBindings(driverController);

    if (!hasTuningEnabled()) {
      configureButtonBoxBindings(buttonBox);
      configureOperatorBindings(operatorController);
    } else {
      configureTestButtonBoxBindings(buttonBox);
      configureTestOperatorBindings(operatorController);
    }

    configureDefaultCommands();
    configureNamedCommands();
    configureEventTriggers();
    configureTriggers(driverController, operatorController);

    // Configure the autonomous path chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos/Pathplanner Trajectory", autoChooser);
  }

  // Public method to provide access to the singleton instance
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  // Public method to check if debug is enabled
  public static boolean hasTracesEnabled() {
    return RobotConstants.tracesEnabled;
  }
  // Public method to check if tuning is enabled
  public static boolean hasTuningEnabled() {
    return RobotConstants.tuningEnabled;
  }

  // ------------------------------------------ MAIN CONTROLLER -----------------------
  // ----------------------------------------------------------------------------------
  private void configureDriverBindings(CS_XboxController controller) {

    // ---------------------------------------- Right Bumper
    //                                          Coral Intake
    controller.btn_RightBumper.toggleOnTrue(
        Commands.defer((() -> new CoralShooterIntake()), Set.of(mortar)));

    // ---------------------------------------- Right Trigger
    //                                          Coral Shoot
    controller.btn_RightTrigger.toggleOnTrue(
        Commands.defer((() -> new ToCoralShoot()), Set.of(mortar)));

    // ---------------------------------------- Left Bumper
    //                                          Algae Intake (based on Dashboard Selection)
    controller.btn_LeftBumper.toggleOnTrue(
        Commands.defer(
            () ->
                new ToPathAndFinkleAndAlgaeIntake()
                    .onlyIf(() -> !algae501.isLoaded())
                    .handleInterrupt(
                        () ->
                            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW)
                                .alongWith(new InstantCommand(() -> algae501.stopAll()))
                                .schedule()),
            Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- Left Trigger
    //                                          Algae Shoot Preset from Barge (low)
    controller.btn_LeftTrigger.toggleOnTrue(
        Commands.defer(
            (() ->
                new ToPathAndFinkleAndAlgaeShoot(() -> PresetManager.getBargeShootPreset())
                    .onlyIf(() -> algae501.isLoaded())
                    .handleInterrupt(
                        () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
            Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- A Button
    //                                          Coral Intake
    controller.btn_A.whileTrue(
        Commands.defer((() -> new ToPathAndFinkleAndCoralIntake()), Set.of(mortar)));

    // ---------------------------------------- B Button
    //                                          Coral Shoot
    controller.btn_B.whileTrue(
        Commands.defer((() -> new ToPathAndFinkleAndCoralShoot()), Set.of(drivebase, mortar)));

    // ---------------------------------------- X Button
    //                                          Discard Algae
    controller.btn_X.whileTrue(
        new ToSubsystemsPreset(() -> Presets.ALGAE_DISCARD)
            .andThen(new AlgaeShooterDiscard())
            .finallyDo(
                interrupted -> {
                  new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
                }));

    // ---------------------------------------- Y Button
    //                                          Stow Algae Manipulator
    controller.btn_Y.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));

    // ---------------------------------------- Back Button
    //                                          Flip Drivebase directon
    controller.btn_Back.onTrue(new InstantCommand(() -> drivebase.flipToggle()));

    // ---------------------------------------- Our/Their Side Triggers
    Trigger ourSide = new Trigger(() -> drivebase.onOurSide());
    Trigger theirSide = new Trigger(() -> !drivebase.onOurSide());

    // ---------------------------------------- POV UP Button
    //                                          Algae Shoot High From Reef
    controller
        .btn_North
        .and(ourSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_SHOOTBARGE_OURSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));

    controller
        .btn_North
        .and(theirSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_SHOOTBARGE_THEIRSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));

    // ---------------------------------------- POV DOWN Button
    //                                          Algae Shoot Low From Reef
    controller
        .btn_South
        .and(ourSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_SHOOTLOW_OURSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));

    controller
        .btn_South
        .and(theirSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_SHOOTLOW_THEIRSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));

    // ---------------------------------------- POV LEFT Button
    //                                          Auto Shoot Algae
    // controller.btn_West.toggleOnTrue(
    //     Commands.defer(
    //         (() ->
    //             new ToAlgaeShoot(
    //                     () -> PresetManager.getAimAndShootPreset(() -> drivebase.getPose()))
    //                 .onlyIf(() -> algae501.isLoaded())),
    //         Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- POV Right Button
    //                                          Process Algae
    controller
        .btn_East
        .and(ourSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_PROCESS_OURSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));

    controller
        .btn_East
        .and(theirSide)
        .whileTrue(
            Commands.defer(
                (() ->
                    new ToPathAndFinkleAndAlgaeShoot(
                            () -> AllianceFlipUtil.apply(Presets.ALGAE_PROCESS_THEIRSIDE))
                        .onlyIf(() -> algae501.isLoaded())
                        .handleInterrupt(
                            () -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule())),
                Set.of(drivebase, elevator, wrist, algae501)));
  }

  // ---------------------------------------- TRIGGERS --------------------------------
  // ----------------------------------------------------------------------------------
  private void configureTriggers(CS_XboxController driver, CS_XboxController operator) {
    // ---------------------------------------- TRIGGER Intake on Coral Sensor
    new Trigger(mortar::hasCoral)
        .debounce(0.1)
        .onTrue(new CoralShooterIntake().onlyIf(() -> !mortar.isLoaded()).withTimeout(2.0));

    // ---------------------------------------- TRIGGER STOW on Tipping Over
    new Trigger(
            () ->
                (elevator.getHeight().in(Inches) > 12
                    && (Math.abs(drivebase.getPitch().in(Degrees)) > 5
                        || Math.abs(drivebase.getRoll().in(Degrees)) > 5)))
        // .debounce(0.025)
        .onTrue(
            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW)
                .alongWith(RumbleCommand.longRumble(driver))
                .alongWith(RumbleCommand.longRumble(operator)));

    // ---------------------------------------- TRIGGER RUMBLE
    new Trigger(algae501::isLoaded)
        .debounce(0.1)
        .onTrue(new RumbleCommand(driver, RumbleType.kBothRumble));

    new Trigger(() -> mortar.isLoaded() || mortar.hasCoral())
        .debounce(0.1)
        .onTrue(new RumbleCommand(driver, RumbleType.kBothRumble));
  }

  // ------------------------------------ OPERATOR CONTROLLER -------------------------
  // ----------------------------------------------------------------------------------
  private void configureOperatorBindings(CS_XboxController controller) {

    // ---------------------------------------- POV UP/DOWN
    //                                          Elevator up/down 1"
    controller.btn_North.onTrue(new InstantCommand(() -> elevator.goUp(Inches.of(1))));
    controller.btn_South.onTrue(new InstantCommand(() -> elevator.goDown(Inches.of(1))));

    // ---------------------------------------- POV LEFT/RIGHT
    //                                          Wrist up/down 5"
    controller.btn_West.onTrue(new InstantCommand(() -> wrist.goUp(Degrees.of(5))));
    controller.btn_East.onTrue(new InstantCommand(() -> wrist.goDown(Degrees.of(5))));

    // ---------------------------------------- X Button
    //                                          Zero Elevator
    controller.btn_X.onTrue(new InstantCommand(() -> elevator.reset()));

    // ---------------------------------------- Y Button
    //                                          Stow Algae Manipulator
    controller.btn_Y.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));

    // ---------------------------------------- Back Button
    //                                          Flip Drivebase directon
    controller.btn_Back.onTrue(new InstantCommand(() -> drivebase.flipToggle()));

    // ---------------------------------------- Elevator Characterization
    //
    // Trigger sysIdQuasistatic = controller.btn_Start;
    // Trigger sysIdDynamic = controller.btn_Back;
    // Trigger sysIdForward = controller.btn_A;
    // Trigger sysIdBack = controller.btn_B;
    // sysIdQuasistatic.and(sysIdForward).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // sysIdQuasistatic.and(sysIdBack).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    // sysIdDynamic.and(sysIdForward).whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // sysIdDynamic.and(sysIdBack).whileTrue(elevator.sysIdDynamic(Direction.kReverse));

  }

  private void configureTestOperatorBindings(CS_XboxController controller) {}

  // ---------------------------------------- BUTTON BOX ------------------------------
  //
  //           +-----------------+
  //           |  1  |  2  |  3  |
  //           |-----+-----+-----|
  //           |  4  |  5  |  6  |
  //           |-----+-----+-----|
  //           |  7  |  8  |  9  |
  //           +-----------------+
  //
  // ----------------------------------------------------------------------------------
  private void configureButtonBoxBindings(CS_ButtonBoxController controller) {}

  private void configureTestButtonBoxBindings(CS_ButtonBoxController controller) {
    controller.btn_1.onTrue(
        Commands.defer(() -> new DriveTurnToAngle(() -> Degrees.of(0)), Set.of()));
    controller.btn_2.onTrue(
        Commands.defer(() -> new DriveTurnToAngle(() -> Degrees.of(90)), Set.of()));
    controller.btn_3.onTrue(
        Commands.defer(() -> new DriveTurnToAngle(() -> Degrees.of(180)), Set.of()));
    controller.btn_4.onTrue(
        Commands.defer(() -> new DriveTurnToAngle(() -> Degrees.of(270)), Set.of()));
  }

  // ---------------------------------------- DEFAULT COMMANDS ------------------------
  // ----------------------------------------------------------------------------------
  private void configureDefaultCommands() {
    drivebase.setDefaultCommand(driverController);
  }

  // ---------------------------------------- NAMED COMMANDS (PATHPLANNER AUTOS) ------
  // ----------------------------------------------------------------------------------
  private void configureNamedCommands() {
    Supplier<CoralPreset> presetSupplier = () -> Presets.CORAL_L4;
    NamedCommands.registerCommand(
        "Intake", new CoralShooterIntake().onlyIf(() -> !mortar.isLoaded()));
    NamedCommands.registerCommand("Shoot", new ToCoralShoot(presetSupplier));
    NamedCommands.registerCommand("FinkleAndShootA", new Auto_A());
    NamedCommands.registerCommand("FinkleAndShootB", new Auto_B());
    NamedCommands.registerCommand("FinkleAndShootC", new Auto_C());
    NamedCommands.registerCommand("FinkleAndShootD", new Auto_D());
    NamedCommands.registerCommand("FinkleAndShootE", new Auto_E());
    NamedCommands.registerCommand("FinkleAndShootF", new Auto_F());
    NamedCommands.registerCommand("FinkleAndShootG", new Auto_G());
    NamedCommands.registerCommand("FinkleAndShootH", new Auto_H());
    NamedCommands.registerCommand("FinkleAndShootI", new Auto_I());
    NamedCommands.registerCommand("FinkleAndShootJ", new Auto_J());
    NamedCommands.registerCommand("FinkleAndShootK", new Auto_K());
    NamedCommands.registerCommand("FinkleAndShootL", new Auto_L());
    NamedCommands.registerCommand("FinkleAndTake4", new Auto_4());
    NamedCommands.registerCommand("FinkleAndTake5", new Auto_5());
    NamedCommands.registerCommand(
        "RemoveSelectedAlgae",
        Commands.defer(
            (() -> new ToPathAndFinkleAndAlgaeIntake().onlyIf(() -> !algae501.isLoaded())),
            Set.of(elevator, wrist, algae501)));
  }

  // ---------------------------------------- EVENT TRIGGERS (PATHPLANNER PATHS) ------
  // ----------------------------------------------------------------------------------
  private void configureEventTriggers() {
    // new EventTrigger("PrepareShootLow").whileTrue(Commands.print("running intake"));

    new EventTrigger("PrepareShootLow")
        .whileTrue(
            Commands.defer(
                () -> new ToSubsystemsPreset(() -> Presets.ALGAE_SHOOTLOW_OURSIDE), Set.of()));

    new EventTrigger("Stow")
        .whileTrue(
            Commands.defer(() -> new ToSubsystemsPreset(() -> Presets.ALGAE_STOW), Set.of()));

    new EventTrigger("PrepareRampUp")
        .and(algae501::isLoaded)
        .whileTrue(
            Commands.defer(
                () ->
                    new AlgaeShooterRampUp(() -> Presets.ALGAE_SHOOTLOW_OURSIDE.getRPM())
                        .withDoNotStopOnInterrupt(),
                Set.of()));

    new EventTrigger("ShootIt")
        .and(algae501::isLoaded)
        .onTrue(
            Commands.defer(
                (() -> new ToAlgaeShoot(() -> Presets.ALGAE_SHOOTLOW_OURSIDE)), Set.of()));
  }

  // ---------------------------------------- PICK AUTONOMOUS COMMAND/TRAJECTORY ------
  // ----------------------------------------------------------------------------------
  public Command getAutonomousCommand() {
    // Return the path to follow in autonomous mode
    Command retVal = new InstantCommand();

    AutoOptions autoOption = dashboard.getSelectedAuto();
    switch (autoOption) {
      case G:
        retVal = new Auto_G();
        break;

      case Practice_L:
        retVal = new Auto_L();
        break;

      case DO_NOTHING:
        retVal = new InstantCommand();
        break;

      case TRAJECTORY:
        retVal = this.autoChooser.getSelected();
        break;
    }
    return retVal;
  }

  public RobotType getRobotType() {
    return RobotConstants.robotType;
  }

  private void displayCredits() {

    System.out.println("");
    System.out.println(
        "####################################################################################");
    System.out.println(
        "###   ___     _               ___       _ _                    ___   __ ___  __  ###");
    System.out.println(
        "###  / __|  _| |__  ___ _ _  / __| __ _(_) |___ _ _ ___  ___  ( _ ) / /|_  )/ /  ###");
    System.out.println(
        "### | (_| || | '_ \\/ -_) '_| \\__ \\/ _` | | / _ \\ '_(_-< |___| / _ \\/ _ \\/ // _ \\ ###");
    System.out.println(
        "###  \\___\\_, |_.__/\\___|_|   |___/\\__,_|_|_\\___/_| /__/       \\___/\\___/___\\___/ ###");
    System.out.println(
        "###      |__/                                                                    ###");
    System.out.println(
        "###                                                                              ###");
    System.out.printf("### %-76s ###\n", "Compliled for: " + RobotConstants.robotType.toString());
    System.out.printf(
        "### %-76s ###\n", "Debug        : " + (hasTracesEnabled() ? "ENABLED" : "DISABLED"));
    System.out.printf(
        "### %-76s ###\n", "Tuning       : " + (hasTuningEnabled() ? "ENABLED" : "DISABLED"));
    System.out.println(
        "###                                                                              ###");
    System.out.printf("### %-76s ###\n", "Repository   : " + BuildConstants.MAVEN_NAME);
    System.out.printf("### %-76s ###\n", "Version      : " + BuildConstants.VERSION);
    System.out.printf("### %-76s ###\n", "Git Branch   : " + BuildConstants.GIT_BRANCH);
    System.out.printf("### %-76s ###\n", "Build Date   : " + BuildConstants.BUILD_DATE);
    System.out.printf(
        "### %-76s ###\n",
        "Dirty Flag   : "
            + (BuildConstants.DIRTY == 0 ? "0- All changes commited" : "1- Uncomitted changes"));
    System.out.println(
        "###                                                                              ###");
    System.out.println(
        "####################################################################################");
    System.out.println("");
  }
}
