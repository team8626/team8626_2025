package frc.robot.subsystems.dummy;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;


public class DummyIO_Specific1 implements DummyIO{
    // Declare the subsystem specific hardware here
    // Example:
     private final CANSparkMax motor1;
     private final AbsoluteEncoder encoder1;
     private final SparkPIDController pidController1;

    public DummyIO_Specific1(){
        // Instantiante and initilaze the substem according to Specific1 configuration.
        // This is specific to that robot (Motors, Sensors, controllers...)
        // Example:
        motor1 = new CANSparkMax(DummyConstants.kvalue1, MotorType.kBrushless);
        encoder1 = motor1.getAbsoluteEncoder(Type.kDutyCycle);
        pidController1 = motor1.getPIDController();
    }

    @Override
    public void updateInputs(DummyIO_Inputs inputs){
        // inputs.subsystemNameInput1 = 0.0;
        // inputs.subsystemNameInput2 = 0.0;
        // inputs.subsystemNameInput3 = true;
    }

    @Override
    public void setValue1(double value1) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setValue1'");
    }
    @Override
    public double getValue1() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getValue1'");
    }
}