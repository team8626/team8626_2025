package frc.robot.subsystems.swervedrive;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OperatorConstants;

public class CS_DriveSubsystemIO_Swerve extends SwerveSubsystem implements CS_DriveSubsystemIO {
    
    public CS_DriveSubsystemIO_Swerve(File directory){
        super(directory);
    }
    /**
     * Sets the drive command for the swerve subsystem using the provided Xbox controller.
     * The command uses the left joystick for forward/backward and strafe movements,
     * and the right joystick for rotation.
     *
     * @param xboxController The Xbox controller to use for driving the robot.
     */
    public void setDefaultCommand(CommandXboxController xboxController){
        Command driveCommand = this.driveCommand(
                () ->
                    MathUtil.applyDeadband(-xboxController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () ->
                    MathUtil.applyDeadband(-xboxController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -xboxController.getRightX());

        setDefaultCommand(driveCommand);
    }
}
