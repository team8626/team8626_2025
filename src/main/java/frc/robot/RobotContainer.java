package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants.RobotType;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Swerve;
import frc.robot.subsystems.drive.CS_DriveSubsystemIO_Tank;
import frc.robot.subsystems.ledManager.LEDManager;

public class RobotContainer {
    // Instantiate the Commodore running the robot state machine. 
    private Commodore commodore = Commodore.getInstance();

    // Instantiate the LED manager 
    private LEDManager ledManager = LEDManager.getInstance();


    // ****************************************************************************************
    // Change robot type here if needed
    // Possible options: SIMBOT, KITBOT, DART, DEVBOT, COMPBOT
    //
    private RobotType robotType = RobotType.DART;
    //
    // ****************************************************************************************

    // Define subsystems and commands here
    // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    // private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
    private CS_DriveSubsystemIO drivebase; 
    
    private final CommandXboxController xboxController =
        new CommandXboxController(OperatorConstants.kXboxControllerPort);

    private SendableChooser<Command> autoChooser;

//   private final CommandXboxController m_testController =
//       new CommandXboxController(OperatorConstants.kTestControllerPort);

//   private final CommandButtonController m_buttonBox =
//       new CommandButtonController(OperatorConstants.kButtonBoxPort);

    public RobotContainer() {

        // Define Robot Subsystems
        if(Robot.isSimulation()) robotType = RobotConstants.RobotType.SIMBOT;

        switch(robotType){
            case KITBOT:
                drivebase = new CS_DriveSubsystemIO_Tank();
                break;
            case SIMBOT:
            case DART:
                drivebase = new CS_DriveSubsystemIO_Swerve(new File(Filesystem.getDeployDirectory(),
                                                            "swerve_dart"));
                break;
            case DEVBOT:
            case COMPBOT:
            default:   
                drivebase = new CS_DriveSubsystemIO_Swerve(new File(Filesystem.getDeployDirectory(),
                                                             "swerve_devbot"));
                break;
        }

        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();

        // Configure the autonomous path chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);
    }

    private void configureButtonBindings() {
   
        // Bind the example command to the example button
        // exampleButton.whenPressed(exampleCommand);
        
    }
    
    private void configureDefaultCommands () {
        // Set the default command for a subsystem here.
            // TODO: @NickCanCode do we need invert for red? 
            // TODO: @NickCanCode implement slow drive in swerve subsystem

        drivebase.setDefaultCommand(xboxController);
    }

    public Command getAutonomousCommand() {
        // Return the path to follow in autonomous mode
        return this.autoChooser.getSelected();
    }
}