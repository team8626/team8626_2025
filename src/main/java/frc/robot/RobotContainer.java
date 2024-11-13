package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
    // Define subsystems and commands here
    // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    // private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);

    // Define joystick and buttons
    private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve_dart"));

  private final CommandXboxController m_xboxController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);

//   private final CommandXboxController m_testController =
//       new CommandXboxController(OperatorConstants.kTestControllerPort);

//   private final CommandButtonController m_buttonBox =
//       new CommandButtonController(OperatorConstants.kButtonBoxPort);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
   
        // Bind the example command to the example button
        // exampleButton.whenPressed(exampleCommand);
        
    }
    private void configureDefaultCommands () {
        // Set the default command for a subsystem here.
        // drivebase.setDefaultCommand(new DriveCommand(drivebase, driverJoystick));
         Command driveFieldOrientedAnglularVelocity =
        m_drivebase.driveCommand(
            () ->
                MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
            () ->
                MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
            () -> -m_xboxController.getRightX());



            // TODO: @NickCanCode do we need invert for red? 
            // TODO: @NickCanCode implement slow drive in swerve subsystem
        //              Command driveFieldOrientedAnglularVelocity =
        // m_drivebase.driveCommand(
        //     () ->
        //         MathUtil.applyDeadband(-m_xboxController.getLeftY() * invert, 0.1)
        //             * driveSpeedFactor,
        //     () ->
        //         MathUtil.applyDeadband(-m_xboxController.getLeftX() * invert, 0.1)
        //             * driveSpeedFactor,
        //     () -> -m_xboxController.getRightX() * rotationSpeedFactor);

  m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAnglularVelocity
            : driveFieldOrientedAnglularVelocity);

    }

    public Command getAutonomousCommand() {
        // Return the command to run in autonomous
        return null;
        // TODO: @NickCanCode Replace null with the command to run in autonomous
    }
}