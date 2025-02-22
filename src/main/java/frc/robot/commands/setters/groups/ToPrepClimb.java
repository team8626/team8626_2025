// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.ClimberSetAngle;
import frc.robot.commands.setters.units.ElevatorSetHeight;
import frc.robot.commands.setters.units.WristSetAngle;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ToPrepClimb extends SequentialCommandGroup {

  public ToPrepClimb() {
    ElevatorSubsystem elevator = RobotContainer.elevator;
    WristSubsystem wrist = RobotContainer.wrist;
    AlgaeShooterSubsystem algae501 = RobotContainer.algae501;
    CoralShooterSubsystem mortar = RobotContainer.mortar;
    // ClimberSubsystem climber = ClimberSubsystem.climber;

    System.out.println("[Cmd: TOPREPCLIMB]");

    addCommands(
        new ParallelCommandGroup(
            Commodore.getSetStateCommand(CommodoreState.CLIMB_PREP),
            new ElevatorSetHeight(() -> 0 /* TODO: GET THE PRESET POSITION */),
            new WristSetAngle(() -> 0 /* TODO: GET THE PRESET POSITION */)),
        new ClimberSetAngle(() -> 0 /* TODO: GET THE PRESET POSITION */),
        new ParallelCommandGroup(
            new ElevatorSetHeight(() -> 0 /* TODO: GET THE PRESET POSITION */),
            new WristSetAngle(() -> 0 /* TODO: GET THE PRESET POSITION */)),
        Commodore.getSetStateCommand(CommodoreState.CLIMB_READY));
  }
}
