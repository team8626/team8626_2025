// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for using button box with triggers */
public class CS_ButtonBoxController extends CommandGenericHID {
  private Joystick m_hid = null;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CS_ButtonBoxController(int port) {
    super(port);
    m_hid = new Joystick(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public Joystick getHID() {
    return m_hid;
  }

  public Trigger btn_1 = new Trigger(() -> m_hid.getRawButton(1));
  public Trigger btn_2 = new Trigger(() -> m_hid.getRawButton(2));
  public Trigger btn_3 = new Trigger(() -> m_hid.getRawButton(3));
  public Trigger btn_4 = new Trigger(() -> m_hid.getRawButton(4));
  public Trigger btn_5 = new Trigger(() -> m_hid.getRawButton(5));
  public Trigger btn_6 = new Trigger(() -> m_hid.getRawButton(6));
  public Trigger btn_7 = new Trigger(() -> m_hid.getRawButton(7));
  public Trigger btn_8 = new Trigger(() -> m_hid.getRawButton(8));
  public Trigger btn_9 = new Trigger(() -> m_hid.getRawButton(9));

  /**
   * Constructs event instance around the button's digital signal.
   *
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger button_1() {
    return m_hid
        .button(1, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_2() {
    return m_hid
        .button(2, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_3() {
    return m_hid
        .button(3, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_4() {
    return m_hid
        .button(4, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_5() {
    return m_hid
        .button(5, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_6() {
    return m_hid
        .button(6, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_7() {
    return m_hid
        .button(7, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_8() {
    return m_hid
        .button(8, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_9() {
    return m_hid
        .button(9, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }
}
