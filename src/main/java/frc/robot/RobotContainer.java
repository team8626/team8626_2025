// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants.RobotType;
import frc.robot.UIConstants.CORAL_BRANCH;
import frc.robot.UIConstants.CORAL_LEVEL;
import frc.robot.commands.RumbleCommand;
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
import frc.robot.commands.setters.groups.ToAlgaePresetAndShoot;
import frc.robot.commands.setters.groups.ToAlgaePresetDriveAndShoot;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.groups.ToPathAndCoralIntake;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndAlgaeShoot;
import frc.robot.commands.setters.groups.ToPathAndFinkleAndCoralShoot;
import frc.robot.commands.setters.groups.ToPathAndFinleAndAlgaeIntake;
import frc.robot.commands.setters.groups.ToSubsystemsPreset;
import frc.robot.commands.setters.units.AlgaeShooterDiscard;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.AutoOptions;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_Sim;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_SparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooter_Sim;
import frc.robot.subsystems.coralshooter.CoralShooter_SparkMax;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.Elevator_LinearSparkMax;
import frc.robot.subsystems.elevator.Elevator_SimulationRose;
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
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;

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
  public static ClimberSubsystem climber = null;

  // Controllers
  public static final CS_XboxController driverController =
      new CS_XboxController(OperatorConstants.DRIVER_CONTOLLER_PORT);
  private static final CS_XboxController operatorController =
      new CS_XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private static final CS_ButtonBoxController buttonBox =
      new CS_ButtonBoxController(OperatorConstants.BUTTON_BOX_PORT);

  private SendableChooser<Command> autoChooser;

  private RobotContainer() {

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

        elevator = new ElevatorSubsystem(new Elevator_SimulationRose());
        wrist = new WristSubsystem(new Wrist_Sim());
        algae501 = new AlgaeShooterSubsystem(new AlgaeShooter_Sim());
        mortar = new CoralShooterSubsystem(new CoralShooter_Sim());
        // visualization = Visualization.getInstance();

        break;
      case COMPBOT:
      default:
        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_devbot"));
        elevator = new ElevatorSubsystem(new Elevator_LinearSparkMax());
        wrist = new WristSubsystem(new Wrist_SparkFlex());
        mortar = new CoralShooterSubsystem(new CoralShooter_SparkMax());
        algae501 = new AlgaeShooterSubsystem(new AlgaeShooter_SparkMax());

        break;
    }

    // Display build information
    displayCredits();

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

  // ---------------------------------------- MAIN CONTROLLER -------------------------
  // ----------------------------------------------------------------------------------
  //
  private void configureDriverBindings(CS_XboxController controller) {

    // ---------------------------------------- Right Bumper
    //                                          Coral Intake
    controller.btn_RightBumper.toggleOnTrue(
        Commands.defer((() -> new ToPathAndCoralIntake()), Set.of(mortar)));
    // controller.btn_RightBumper.toggleOnTrue(new ToCoralIntake());

    // ---------------------------------------- Right Trigger
    //                                          Coral Shoot
    // controller.btn_RightTrigger.toggleOnTrue(new ToPathAndCoralShoot3());
    controller.btn_RightTrigger.toggleOnTrue(
        Commands.defer((() -> new ToPathAndFinkleAndCoralShoot()), Set.of(drivebase, mortar)));

    // ---------------------------------------- Left Bumper
    //                                          Algae Intake (based on Dashboard Selection)
    controller.btn_LeftBumper.toggleOnTrue(
        Commands.defer(
            (() -> new ToPathAndFinleAndAlgaeIntake().onlyIf(() -> !algae501.isLoaded())),
            Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- Left Trigger
    //                                          Algae Shoot Preset from Barge (low)
    controller.btn_LeftTrigger.toggleOnTrue(
        Commands.defer(
            (() ->
                new ToPathAndFinkleAndAlgaeShoot(
                        () -> PresetManager.getBargeShootPreset(() -> drivebase.getPose()))
                    .onlyIf(() -> algae501.isLoaded())),
            Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- Y Button
    //                                          Stow Algae Manipulator
    controller.btn_Y.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));

    // ---------------------------------------- X Button
    //                                          Discard Algae
    controller.btn_X.toggleOnTrue(
        new ToSubsystemsPreset(() -> Presets.ALGAE_PROCESS)
            .andThen(new AlgaeShooterDiscard())
            .finallyDo(
                interrupted -> {
                  new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
                }));

    // ---------------------------------------- B Button
    //                                          Algae Process
    controller.btn_B.toggleOnTrue(
        Commands.defer(
                (() -> new ToAlgaePresetAndShoot(() -> Presets.ALGAE_PROCESS)),
                Set.of(elevator, wrist, algae501))
            .onlyIf(algae501::isLoaded));

    // ---------------------------------------- A Button
    //                                          Algae Shoot High From Reef
    controller.btn_A.toggleOnTrue(
        Commands.defer(
                (() -> new ToAlgaePresetAndShoot(() -> Presets.ALGAE_NETFROMREEF)),
                Set.of(elevator, wrist, algae501))
            .onlyIf(algae501::isLoaded));

    // ---------------------------------------- Back Button
    //                                          Flip Drivebase directon
    controller.btn_Back.onTrue(new InstantCommand(() -> drivebase.flipToggle()));

    // ---------------------------------------- POV UP/DOWN Button
    //                                          Algae Shoot High From Reef
    controller.btn_South.toggleOnTrue(
        Commands.defer(
                (() -> new ToAlgaePresetDriveAndShoot(() -> Presets.ALGAE_SHOOTBARGE_OURSIDE)),
                Set.of(elevator, wrist, algae501))
            .onlyIf(algae501::isLoaded));

    controller.btn_North.toggleOnTrue(
        Commands.defer(
                (() -> new ToAlgaePresetDriveAndShoot(() -> Presets.ALGAE_SHOOTBARGE_THEIRSIDE)),
                Set.of(elevator, wrist, algae501))
            .onlyIf(algae501::isLoaded));

    controller.btn_East.toggleOnTrue(new ToCoralShoot());

    // ---------------------------------------- POV UP/DOWN/LEFT/RIGHT -- FOR TESTING ONLY
    // controller.btn_South.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_NETFROMREEF));
    // controller.btn_East.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_FLOOR));
    // controller.btn_North.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));
    // controller.btn_West.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_SHOOTBARGE_OURSIDE));

    // ---------------------------------------- TRIGGER RUMBLE
    new Trigger(algae501::isLoaded)
        .debounce(0.1)
        .onTrue(new RumbleCommand(controller, RumbleType.kBothRumble));

    new Trigger(mortar::isLoaded)
        .debounce(0.1)
        .onTrue(new RumbleCommand(controller, RumbleType.kBothRumble));

    new Trigger(() -> Math.abs(drivebase.getPitch() - 10) > 0)
        .onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));
  }

  // ------------------------------------ OPERATOR CONTROLLER -------------------------
  // ----------------------------------------------------------------------------------
  private void configureOperatorBindings(CS_XboxController controller) {

    Supplier<Pose2d> targetPose =
        () -> PresetManager.getRobotPoseFromTarget(CORAL_BRANCH.G, CORAL_LEVEL.L4, 0);
    Supplier<Pose2d> offsetPose =
        () -> PresetManager.getRobotPoseFromTarget(CORAL_BRANCH.G, CORAL_LEVEL.L4, 12);

    controller.btn_Y.toggleOnTrue(new Auto_G());
    controller.btn_A.toggleOnTrue(new DriveToPoseFinkle(offsetPose));
    controller.btn_B.toggleOnTrue(new DriveToPoseFinkle(targetPose));

    // controller.btn_A.toggleOnTrue(
    //     Commands.defer(
    //         (() ->
    //             new ToPathAndFinkleAndAlgaeShoot(
    //                     PresetManager.getBargeShootPreset(() -> drivebase.getPose()))
    //                 .onlyIf(() -> algae501.isLoaded())),
    //         Set.of(elevator, wrist, algae501)));

    // ---------------------------------------- POV UP/DOWN
    //                                          Elevator up/down 1"
    controller.btn_North.onTrue(new InstantCommand(() -> elevator.goUp(1.0)));
    controller.btn_South.onTrue(new InstantCommand(() -> elevator.goDown(1.0)));

    // ---------------------------------------- POV LEFT/RIGHT
    //                                          Wrist up/down 5"
    controller.btn_West.onTrue(new InstantCommand(() -> wrist.goUp(5.0)));
    controller.btn_East.onTrue(new InstantCommand(() -> wrist.goDown(5.0)));

    // ---------------------------------------- X Button
    //                                          Zero Elevator
    controller.btn_X.onTrue(new InstantCommand(() -> elevator.reset()));

    // ---------------------------------------- Back Button
    //                                          Flip Drivebase directon
    controller.btn_Back.onTrue(new InstantCommand(() -> drivebase.flipToggle()));
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
  private void configureButtonBoxBindings(CS_ButtonBoxController controller) {

    // Supplier<Pose2d> targetPose = () -> PresetManager.getCoralPreset().getPose();
    // Supplier<Pose2d> offsetPose =
    //     () ->
    //         PresetManager.getCoralPreset()
    //             .getPose()
    //             .plus(new Transform2d(Units.inchesToMeters(36), 0, new Rotation2d()));

    // controller.btn_1.toggleOnTrue(
    //     new FollowPathToPose(() -> PresetManager.getCoralPreset().getPose(), () -> 36)
    //         .andThen(new DriveToPoseFinkle(() -> PresetManager.getCoralPreset().getPose()))
    //         .andThen(new ToCoralShoot3()));

    // controller.btn_2.toggleOnTrue(
    //     new FollowPathToPose(() -> PresetManager.getCoralPreset().getPose(), () -> 36));

    // controller.btn_3.toggleOnTrue(
    //     new DriveToPoseFinkle(() -> PresetManager.getCoralPreset().getPose())
    //         .andThen(new ToCoralShoot3()));

    // controller.btn_4.toggleOnTrue(
    //     Commands.defer((() -> new ToPathAndFinkleAndCoralShoot()), Set.of(drivebase, mortar)));

    // controller.btn_5.toggleOnTrue(new FollowPathToPose(offsetPose));

    // controller.btn_6.toggleOnTrue(new DriveToPoseFinkle(targetPose).andThen(new
    // ToCoralShoot3()));

    // Pose2d testPose = new Pose2d(3, 2, new Rotation2d(Units.degreesToRadians(-120)));
    // controller.btn_9.toggleOnTrue(new DriveToPoseFinkle(() -> testPose));

    // controller.btn_2.toggleOnTrue(
    //     new DriveToPoseFinkle(() -> PresetManager.getCoralPreset().getPose(), true));

    // controller.btn_3.toggleOnTrue(
    //     drivebase.driveToPose(() -> PresetManager.getCoralPreset().getPose(24)));

    // controller.btn_4.toggleOnTrue(
    //     new ToSubsystemsPreset(() -> Presets.ALGAE_Test1)
    //         .andThen(new ToAlgaeShoot(() -> Presets.ALGAE_Test1.getRPM()))
    //         .onlyIf(algae501::isLoaded)
    //         .finallyDo(
    //             interrupted -> {
    //               new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
    //             }));

    // controller.btn_5.toggleOnTrue(
    //     drivebase
    //         .driveToPose(() -> PresetManager.getCoralPreset().getPose(24))
    //         // .andThen(drivebase.reset())
    //         .andThen(new DriveToPoseFinkle(() -> PresetManager.getCoralPreset().getPose(), true))
    //         .andThen(new ToCoralShoot3()));

    // controller.btn_7.toggleOnTrue(
    //     new ToSubsystemsPreset(() -> Presets.ALGAE_Test2)
    //         .andThen(new ToAlgaeShoot(() -> Presets.ALGAE_Test2.getRPM()))
    //         .onlyIf(algae501::isLoaded)
    //         .finallyDo(
    //             interrupted -> {
    //               new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
    //             }));

    // controller.btn_8.toggleOnTrue(
    //     new ToSubsystemsPreset(() -> Presets.ALGAE_Test3)
    //         .andThen(new ToAlgaeShoot(() -> Presets.ALGAE_Test3.getRPM()))
    //         .onlyIf(algae501::isLoaded)
    //         .finallyDo(
    //             interrupted -> {
    //               new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
    //             }));
    // controller.btn_9.toggleOnTrue(
    //     new ToSubsystemsPreset(() -> Presets.ALGAE_Test4)
    //         .andThen(new ToAlgaeShoot(() -> Presets.ALGAE_Test4.getRPM()))
    //         .onlyIf(algae501::isLoaded)
    //         .finallyDo(
    //             interrupted -> {
    //               new ToSubsystemsPreset(() -> Presets.ALGAE_STOW).schedule();
    //             }));
  }

  private void configureTestButtonBoxBindings(CS_ButtonBoxController controller) {
    controller.btn_7.toggleOnTrue(
        new FeedForwardCharacterization(
            mortar, mortar::runCharacterizationLeft, mortar::getCharacterizationVelocityLeft));
    controller.btn_8.toggleOnTrue(
        new FeedForwardCharacterization(
            mortar, mortar::runCharacterizationRight, mortar::getCharacterizationVelocityRight));
    controller.btn_9.toggleOnTrue(
        new FeedForwardCharacterization(
            algae501, algae501::runCharacterization, algae501::getCharacterizationVelocity));
  }

  private void configureDefaultCommands() {
    drivebase.setDefaultCommand(driverController);
  }

  private void configureNamedCommands() {
    Supplier<CoralPreset> presetSupplier = () -> Presets.CORAL_L4;
    NamedCommands.registerCommand("Intake", new ToCoralIntake());
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
    NamedCommands.registerCommand(
        "RemoveSelectedAlgae",
        Commands.defer(
            (() -> new ToPathAndFinleAndAlgaeIntake().onlyIf(() -> !algae501.isLoaded())),
            Set.of(elevator, wrist, algae501)));
  }

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
