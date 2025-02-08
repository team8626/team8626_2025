package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;

import static frc.robot.subsystems.wrist.WristConstants.gains;
import static frc.robot.subsystems.wrist.WristConstants.wristConfig;

public class Wrist_Sim implements WristInterface, CS_InterfaceBase {

  private boolean isEnabled = false;

  private double currentLauncherSetpoint = 0;

   // Standard classes for controlling our arm
   private final SingleJointedArmSim m_armSim;
   private final EncoderSim m_encoderSim;


  private final PIDController m_controller = new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final Encoder m_encoder =
      new Encoder(0,1);
  private final PWMSparkMax m_motor = new PWMSparkMax(wristConfig.CANID());

  private DCMotor motor;

  public Wrist_Sim() {
    motor = DCMotor.getNeoVortex(1);
    m_armSim =
      new SingleJointedArmSim(
          motor,
          WristConstants.gearBoxRatio,
          SingleJointedArmSim.estimateMOI(WristConstants.armLength, WristConstants.armMass),
          WristConstants.armLength,
          Units.degreesToRadians(WristConstants.minAngleDegrees),
          Units.degreesToRadians(WristConstants.maxAngleDegrees),
          true,
          0,
          WristConstants.wristEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );
  m_encoderSim = new EncoderSim(m_encoder);
    
  }

  @Override
  public void updateInputs(WristValues values) {

  }

  
  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }

  @Override
  public void setAngleDegrees(double new_Angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setAngleDegrees'");
  }

  @Override
  public double getAngleDegrees() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAngleDegrees'");
  }
}
