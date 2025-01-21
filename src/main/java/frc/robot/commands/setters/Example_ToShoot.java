// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.commands.Dummy3sCommand;

public class Example_ToShoot extends SequentialCommandGroup {

  public Example_ToShoot() {
    System.out.println("[Cmd: EXAMPLE_TOSHOOT]");
    addCommands(
        // new StopAllRollers(),
        // new ParallelCommandGroup(
        //     new LoaderToIntake(),
        //     new ArmToIntake()
        // ),
        // new GrabberToIntake(),
        // new IntakeToIntake(),
        Commodore.getSetStateCommand(CommodoreState.SHOOT),
        new Dummy3sCommand(),
        Commodore.getSetStateCommand(CommodoreState.IDLE));
  }
}
