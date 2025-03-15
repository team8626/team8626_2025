package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterLaunch;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.AlgaeShooterStop;
import frc.robot.commands.setters.units.DriveToPoseFinkle;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToPathAndAlgaeShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();

  public ToPathAndAlgaeShoot(Supplier<AlgaePreset> presetSupplier) {
    AlgaeShooterSubsystem algae501 = RobotContainer.algae501;
    System.out.printf("[Cmd: %s] Loaded\n", this.getName().toUpperCase());

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.ALGAE_SHOOT_RAMPINGUP),
                new ParallelCommandGroup(
                    new DriveToPoseFinkle(() -> presetSupplier.get().getPose(), () -> 5, () -> 5)
                        .onlyIf(() -> PresetManager.usingDtp()),
                    new ToSubsystemsPreset(presetSupplier),
                    new AlgaeShooterRampUp(() -> presetSupplier.get().getRPM()) {
                      @Override
                      public void initialize() {
                        super.initialize();
                        timer.reset();
                        timer.start();
                      }
                    }),
                Commodore.getSetStateCommand(CommodoreState.ALGAE_SHOOT_LAUNCHING),
                new AlgaeShooterLaunch(),
                new AlgaeShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/AlgaeShooter/LastShotIn(ms)", (int) (elapsedTime * 1000));
                    Dashboard.publishAlgaeShootTime((int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE),
                new ToSubsystemsPreset(() -> Presets.ALGAE_STOW)),
            Commodore.getSetStateCommand(CommodoreState.IDLE),
            algae501::isLoaded));
  }
}
