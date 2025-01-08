package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankSubsystem extends SubsystemBase {
  // Declare motor controllers
  private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(31);
  private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(32);
  private final WPI_VictorSPX leftMotor3 = new WPI_VictorSPX(33);
  private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(41);
  private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(42);
  private final WPI_VictorSPX rightMotor3 = new WPI_VictorSPX(43);

  // Declare the differential drive
  private final DifferentialDrive differentialDrive;

  public TankSubsystem() {
    // Set up the differential drive
    rightMotor1.setInverted(true);
    differentialDrive = new DifferentialDrive(leftMotor1, rightMotor1);

    // Set the other motors to follow the leaders
    leftMotor2.follow(leftMotor1);
    leftMotor3.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
    rightMotor3.follow(rightMotor1);
  }

  /**
   * Drives the robot using tank drive controls.
   *
   * @param leftSpeed The speed for the left side of the drive.
   * @param rightSpeed The speed for the right side of the drive.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
