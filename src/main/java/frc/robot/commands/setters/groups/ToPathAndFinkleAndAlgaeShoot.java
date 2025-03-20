package frc.robot.commands.setters.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
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

public class ToPathAndFinkleAndAlgaeShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();

  Supplier<Pose2d> targetPose = () -> PresetManager.getAlgaePreset().get().getShootingPose();
  Supplier<Pose2d> offsetPose =
      () ->
          PresetManager.getAlgaePreset()
              .get()
              .getShootingPose()
              .plus(
                  new Transform2d(
                      Units.inchesToMeters(18), Units.inchesToMeters(-18), new Rotation2d()));
  private Supplier<AlgaePreset> algaePreset;
  AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

  public ToPathAndFinkleAndAlgaeShoot(Supplier<AlgaePreset> presetSupplier) {
    algaePreset = presetSupplier;

    addCommands(
        new SequentialCommandGroup(
                // Drive to Offset Pose
                new SequentialCommandGroup(
                        Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                        AutoBuilder.pathfindToPose(
                            offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.5),
                        Commodore.getSetStateCommand(CommodoreState.IDLE))
                    .onlyIf(() -> !targetPose.get().equals(new Pose2d())),
                Commodore.getSetStateCommand(CommodoreState.IDLE),

                // Drive to Target Pose while setting subsystems
                new ParallelCommandGroup(
                    // Drive to Target Pose
                    new SequentialCommandGroup(new DriveToPoseFinkle(offsetPose))
                        .onlyIf(() -> offsetPose.get().equals(new Pose2d())),

                    // Adjust Subsystems & Start Rampup
                    new ToSubsystemsPreset(algaePreset)),
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
                        "Subsystem/AlgaeShooter/LastShotIn(ms)", (int) (elapsedTime * 1000));
                    Dashboard.publishAlgaeShootTime((int) (elapsedTime * 1000));
                  }
                },

                // Stow
                new ToSubsystemsPreset(() -> Presets.ALGAE_STOW))

            // Handle Interrupts
            .finallyDo(
                (interrupted) ->
                    new SequentialCommandGroup(
                        new InstantCommand(algae501::stopAll),
                        new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
                        Commodore.getSetStateCommand(CommodoreState.IDLE))));
  }
}
