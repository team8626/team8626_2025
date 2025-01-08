// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commodore.CommodoreState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer = null;
  private static boolean isReady = false;

  public Robot() {
    robotContainer = RobotContainer.getInstance();

    // TODO: Remove? We aren't using that m_chooser.set
    // DefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    // log to USB
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Set the Commodore to boot up (disconnected)
    // if(RobotContainer.isDebugEnabled()){
    //
    // DriverStation.reportWarning("################################################################################", false);
    //   DriverStation.reportWarning("########## WARNING !!!
    //        ##########", false);
    //   DriverStation.reportWarning("########## DEBUG IS ENABLED ON THIS BUILD...
    //        ##########", false);
    //
    // DriverStation.reportWarning("################################################################################", false);
    // }
    Commodore.setCommodoreState(CommodoreState.BOOT, true);
    isReady = true;
  }

  /** Returns true of we have been thought the constructor and are ready to run */
  public static boolean isReady() {
    return isReady;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("------------------------------ Autonomous ------------------------------");

    if (DriverStation.isDSAttached() || DriverStation.isFMSAttached()) {
      Commodore.setCommodoreState(CommodoreState.IDLE, true);
    }
    Command autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("------------------------------ Teleop ------------------------------");

    if ((DriverStation.isDSAttached() || DriverStation.isFMSAttached())) {
      Commodore.setCommodoreState(CommodoreState.IDLE, true);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("------------------------------ Disabled ------------------------------");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (!(DriverStation.isFMSAttached() && DriverStation.isDSAttached())) {
      if (Commodore.getCurrentState() != CommodoreState.DISCONNECTED) {
        Commodore.setCommodoreState(CommodoreState.DISCONNECTED, true);
      }
    } else {
      if (Commodore.getCurrentState() != CommodoreState.DISABLED) {
        Commodore.setCommodoreState(CommodoreState.DISABLED, true);
      }
    }
  }

  @Override
  public void disabledExit() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("------------------------------ Test ------------------------------");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    System.out.println("------------------------------ Simulation ------------------------------");
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
