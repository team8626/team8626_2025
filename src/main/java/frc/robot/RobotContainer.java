// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.RobotType;
import frc.robot.commands.setters.autos.Auto_C;
import frc.robot.commands.setters.autos.Auto_G;
import frc.robot.commands.setters.autos.Auto_H;
import frc.robot.commands.setters.autos.Auto_L;
import frc.robot.commands.setters.autos.Auto_PickupLeft;
import frc.robot.commands.setters.autos.Auto_PickupRight;
import frc.robot.commands.setters.groups.ToAlgaeShootFrom10ft;
import frc.robot.commands.setters.groups.ToAlgaeShootFromReef;
import frc.robot.commands.setters.groups.ToAlgaeSpit;
import frc.robot.commands.setters.groups.ToCoralIntake;
import frc.robot.commands.setters.groups.ToCoralShoot;
import frc.robot.commands.setters.groups.ToPathAndCoralShoot;
import frc.robot.commands.setters.groups.ToPathAndCoralShoot2;
import frc.robot.commands.setters.groups.ToPathAndDeAlgaefy;
import frc.robot.commands.setters.groups.ToSubsystemsPreset;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_Sim;
import frc.robot.subsystems.algaeshooter.AlgaeShooter_SparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooter_Sim;
import frc.robot.subsystems.coralshooter.CoralShooter_SparkMax;
import frc.robot.subsystems.drive.CS_DriveSubsystem;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Swerve;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Tank;
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
  public static CS_DriveSubsystem drivebase = null;
  public static ElevatorSubsystem elevator = null;
  public static WristSubsystem wrist = null;
  public static CoralShooterSubsystem mortar = null;
  public static AlgaeShooterSubsystem algae501 = null;
  public static ClimberSubsystem climber = null;

  // Controllers
  private final CS_XboxController driverController =
      new CS_XboxController(OperatorConstants.kXboxControllerPort);
  private final CS_XboxController operatorController =
      new CS_XboxController(OperatorConstants.kTestControllerPort);
  private final CS_ButtonBoxController buttonBox =
      new CS_ButtonBoxController(OperatorConstants.kButtonBoxPort);

  private SendableChooser<Command> autoChooser;

  private RobotContainer() {

    // Define Robot Subsystems
    if (Robot.isSimulation()) {
      RobotConstants.robotType = RobotConstants.RobotType.SIMBOT;
    }

    switch (RobotConstants.robotType) {
      case KITBOT:
        drivebase = new CS_DriveSubsystem(new CS_DriveSubsystemIO_Tank());
        // elevator = new ElevatorArm();
        break;
      case DART:
        drivebase =
            new CS_DriveSubsystem(
                new CS_DriveSubsystemIO_Swerve(
                    new File(Filesystem.getDeployDirectory(), "swerve_dart")));
        break;
      case SIMBOT:
        drivebase =
            new CS_DriveSubsystem(
                new CS_DriveSubsystemIO_Swerve(
                    new File(Filesystem.getDeployDirectory(), "swerve_devbot")));

        elevator = new ElevatorSubsystem(new Elevator_SimulationRose());
        wrist = new WristSubsystem(new Wrist_Sim());
        algae501 = new AlgaeShooterSubsystem(new AlgaeShooter_Sim());
        mortar = new CoralShooterSubsystem(new CoralShooter_Sim());
        // visualization = Visualization.getInstance();

        break;
      case COMPBOT:
      default:
        drivebase =
            new CS_DriveSubsystem(
                new CS_DriveSubsystemIO_Swerve(
                    new File(Filesystem.getDeployDirectory(), "swerve_devbot")));
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

    // Configure the autonomous path chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoChooser);

    // Configure NamedCommands for PathPlanner
    configureNamedCommands();
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

  private void configureDriverBindings(CS_XboxController controller) {
    controller.btn_RightBumper.toggleOnTrue(new ToCoralIntake());
    controller.btn_RightTrigger.toggleOnTrue(new ToPathAndCoralShoot());

    controller.btn_LeftBumper.toggleOnTrue(new ToPathAndDeAlgaefy());
    controller.btn_LeftTrigger.toggleOnTrue(new ToAlgaeShootFrom10ft());

    controller.btn_Y.onTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));
    controller.btn_X.toggleOnTrue(new ToAlgaeSpit());
    controller.btn_B.toggleOnTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_PROCESS));
    controller.btn_A.toggleOnTrue(new ToAlgaeShootFromReef());
  }

  private void configureOperatorBindings(CS_XboxController controller) {
    // controller.btn_A.onTrue(
    //     new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT, true)));
    // controller.btn_A.toggleOnTrue(new ToPathAndDeAlgaefy());

    controller.btn_North.onTrue(new InstantCommand(() -> elevator.goUp(1.0)));
    controller.btn_South.onTrue(new InstantCommand(() -> elevator.goDown(1.0)));

    controller.btn_West.onTrue(new InstantCommand(() -> wrist.goUp(5.0)));
    controller.btn_East.onTrue(new InstantCommand(() -> wrist.goDown(5.0)));

    controller.btn_X.onTrue(new InstantCommand(() -> elevator.reset()));
    // controller.btn_B.onTrue(new Auto_A());

    // controller.btn_A.onTrue(new InstantCommand(() -> elevator.setHeight(10.0)));
    // controller.btn_B.onTrue(new InstantCommand(() -> elevator.setHeight(28.0)));
    // controller.btn_Y.onTrue(new InstantCommand(() -> elevator.setHeight(40.0)));
    // controller.btn_X.onTrue(new InstantCommand(() -> elevator.setHeight(51.0)));
  }

  private void configureTestOperatorBindings(CS_XboxController controller) {}

  // ---------------------------------------- BUTTON BOX ------------------------------
  //
  //           +-----------------+
  //           |  1  |  4  |  7  |
  //           |-----+-----+-----|
  //           |  2  |  5  |  8  |
  //           |-----+-----+-----|
  //           |  3  |  6  |  9  |
  //           +-----------------+
  //
  // ----------------------------------------------------------------------------------
  private void configureButtonBoxBindings(CS_ButtonBoxController controller) {
    // controller.btn_1.toggleOnTrue(new ToAlgaeShootFromReef());

    controller.btn_2.toggleOnTrue(new ToPathAndCoralShoot2());
    // controller.btn_3.toggleOnTrue(new ToPathAndDeAlgaefy());
    // controller.btn_4.toggleOnTrue(
    //     new ToAlgaeShootAuto(() -> PresetManager.getAimAndShootPreset(drivebase.getPose2d())));

    // controller.btn_3.toggleOnTrue(new ToCoralIntake());
    // controller.btn_4.toggleOnTrue(new ToCoralShoot());
    // controller.btn_5.toggleOnTrue(new ToAlgaeShootFromReef());
    // controller.btn_6.toggleOnTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_STOW));
    // controller.btn_7.toggleOnTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_PROCESS));
    // controller.btn_8.toggleOnTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_SHOOT));
    // controller.btn_9.toggleOnTrue(new ToSubsystemsPreset(() -> Presets.ALGAE_INTAKE));
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
    NamedCommands.registerCommand("Shoot", new ToCoralShoot(presetSupplier));
    NamedCommands.registerCommand("Intake", new ToCoralIntake());
  }

  public Command getAutonomousCommand() {
    Command retVal = null;
    switch (dashboard.getSelectedAuto()) {
      case H:
        retVal = new Auto_H();
        break;
      case H_LEFT_L:
        retVal = new SequentialCommandGroup(new Auto_H(), new Auto_PickupLeft(), new Auto_L());
        break;
      case G_RIGHT_C:
        retVal = new SequentialCommandGroup(new Auto_G(), new Auto_PickupRight(), new Auto_C());
        break;
      case DO_NOTHING:
        retVal = null;
        break;
      case TRAJECTORY:
      default:
        autoChooser.getSelected();
    }
    System.out.println("*********************  Selected Auto: " + dashboard.getSelectedAuto());
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
