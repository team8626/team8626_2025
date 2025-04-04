package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterLaunch;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.AlgaeShooterStop;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToAlgaeShootNoStow extends SequentialCommandGroup {
  private final Timer timer = new Timer();

  private Supplier<AlgaePreset> algaePreset;
  AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

  public ToAlgaeShootNoStow(Supplier<AlgaePreset> presetSupplier) {
    algaePreset = presetSupplier;

    addCommands(
        new SequentialCommandGroup(
            // Adjust Subsystems & Start Rampup
            new ToSubsystemsPreset(algaePreset),
            new AlgaeShooterRampUp(() -> algaePreset.get().getRPM()) {
              @Override
              public void initialize() {
                super.initialize();
                timer.reset();
                timer.start();
              }
            },
            // Shoot Algae
            new AlgaeShooterLaunch(),

            // Stop Shooter and Reset
            new AlgaeShooterStop() {
              @Override
              public void initialize() {
                super.initialize();
                double elapsedTime = timer.get();
                SmartDashboard.putNumber(
                    "Subsystem/AlgaeShooterSubsystem/LastShotIn(ms)", (int) (elapsedTime * 1000));
                Dashboard.publishAlgaeShootTime((int) (elapsedTime * 1000));
              }
            },
            Commodore.getSetStateCommand(CommodoreState.IDLE)));
  }
}
