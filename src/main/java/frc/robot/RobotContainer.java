// Copyright (c) 2024 FRC 8626
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
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants.RobotType;
import frc.robot.commands.setters.units.CoralShooterIntake;
import frc.robot.commands.setters.units.CoralShooterLaunch;
import frc.robot.commands.tuning.Tune_CoralShooter;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooter_Sim;
import frc.robot.subsystems.coralshooter.CoralShooter_SparkMax;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Swerve;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Tank;
import frc.robot.subsystems.dummy.DummyIO_Specific1;
import frc.robot.subsystems.dummy.DummySubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.Elevator_LinearSparkMax;
import frc.robot.subsystems.elevator.Elevator_Simulation;
import frc.robot.subsystems.ledManager.LEDManager;
import frc.utils.CS_ButtonBoxController;
import frc.utils.CS_XboxController;
import java.io.File;

public class RobotContainer {
  // Singleton instance
  private static RobotContainer instance;

  // Instantiate the Commodore running the robot state machine.
  private final Commodore commodore = Commodore.getInstance();

  // Instantiate the LED manager
  private final LEDManager ledManager = LEDManager.getInstance();

  //
  // ****************************************************************************************

  // Define subsystems and commands here
  // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
  public static CS_DriveSubsystemIO drivebase = null;
  public static DummySubsystem dummy = null;
  public static ElevatorSubsystem elevator = null;
  public static CoralShooterSubsystem mortar = null;

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
        drivebase = new CS_DriveSubsystemIO_Tank();
        // elevator = new ElevatorArm();
        break;
      case DART:
        dummy = new DummySubsystem(new DummyIO_Specific1());
        drivebase =
            new CS_DriveSubsystemIO_Swerve(
                new File(Filesystem.getDeployDirectory(), "swerve_dart"));
        break;
      case SIMBOT:
        drivebase =
            new CS_DriveSubsystemIO_Swerve(
                new File(Filesystem.getDeployDirectory(), "swerve_devbot"));
        dummy = new DummySubsystem(new DummyIO_Specific1());
        elevator = new ElevatorSubsystem(new Elevator_Simulation());
        mortar = new CoralShooterSubsystem(new CoralShooter_Sim());

        break;
      case DEVBOT:
        mortar = new CoralShooterSubsystem(new CoralShooter_SparkMax());
        drivebase =
            new CS_DriveSubsystemIO_Swerve(
                new File(Filesystem.getDeployDirectory(), "swerve_devbot"));
        break;
      case COMPBOT:
      default:
        drivebase =
            new CS_DriveSubsystemIO_Swerve(
                new File(Filesystem.getDeployDirectory(), "swerve_devbot"));
        dummy = new DummySubsystem(new DummyIO_Specific1());
        elevator = new ElevatorSubsystem(new Elevator_LinearSparkMax());
        mortar = new CoralShooterSubsystem(new CoralShooter_SparkMax());

        break;
    }

    // Display build information
    displayCredits();

    // Configure the button bindings
    configureDriverBindings(driverController);
    configureOperatorBindings(operatorController);

    if (!hasTuningEnabled()) {
      configureButtonBoxBindings(buttonBox);
    } else {
      configureTestButtonBoxBindings(buttonBox);
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
    controller.btn_A.onTrue(
        new InstantCommand(
            () ->
                Commodore.setCommodoreState(CommodoreState.CORAL_INTAKE, true).withToggleState()));

    controller.btn_B.onTrue(
        new InstantCommand(
            () -> Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT, true).withToggleState()));
  }

  private void configureOperatorBindings(CS_XboxController controller) {
    // controller.btn_A.onTrue(
    //     new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT, true)));
  }

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
    // controller.btn_1.onTrue(
    //     new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.INTAKE, true)));
    // controller.btn_9.onTrue(
    //     new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.IDLE, true)));
  }

  private void configureTestButtonBoxBindings(CS_ButtonBoxController controller) {
    // Coral Shooter Tuning functions
    controller.btn_1.toggleOnTrue(new CoralShooterIntake());
    controller.btn_2.toggleOnTrue(new Tune_CoralShooter());
    controller.btn_3.toggleOnTrue(new CoralShooterLaunch());
    // controller.btn_4.toggleOnTrue(
    //     new FeedForwardCharacterization(
    //         mortar, mortar::runCharacterization, mortar::getCharacterizationVelocity));

  }

  private void configureDefaultCommands() {
    // Set the default command for a subsystem here.
    // TODO: @NickCanCode do we need invert for red?
    // TODO: @NickCanCode implement slow drive in swerve subsystem

    drivebase.setDefaultCommand(driverController);
  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand(
        "Idle", new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.IDLE, true)));
    NamedCommands.registerCommand(
        "Shoot", new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.SHOOT, true)));
    NamedCommands.registerCommand(
        "Intake",
        new InstantCommand(() -> Commodore.setCommodoreState(CommodoreState.INTAKE, true)));

    // NamedCommands.registerCommand("Note1_Check", new Note1_Check());
    // NamedCommands.registerCommand("Note2_Check", new Note2_Check());
    // NamedCommands.registerCommand("Note3_Check", new Note3_Check());
  }

  public Command getAutonomousCommand() {
    // Return the path to follow in autonomous mode
    return this.autoChooser.getSelected();
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
        "### %-76s ###\n", "Tuning        : " + (hasTuningEnabled() ? "ENABLED" : "DISABLED"));
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
