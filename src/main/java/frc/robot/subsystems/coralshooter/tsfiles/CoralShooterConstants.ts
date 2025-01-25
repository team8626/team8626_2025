// CoralShooterConstants.ts

import { Units } from 'edu.wpi.first.math.util.Units';
import { RobotConstants } from '../RobotConstants';

// CoralShooterConstants
export const CoralShooterConstants = {
    // Tuned Values
    shootRPM: 1500,
    intakeRPM: -100,
    launcherShootSetpoint: -1.0,
    launcherIntakeSetpoint: 0.3,

    shooterRPMTolerance: 50,
    launchTimerSeconds: 0.2,

    // CoralShooter Constants
    wheelRadiusMeters: Units.inchesToMeters(2),
    wheelMassKg: Units.lbsToKilograms(0.03),
    momentOfInertia: 1,

    // Flywheel Config
    flywheelConfig: (() => {
        switch (RobotConstants.robotType) {
            case 'COMPBOT':
                return new FlywheelConfig(12, 2, 3.0 / 1.0, 2, 6000.0);
            case 'DEVBOT':
                return new FlywheelConfig(12, 2, 3.0 / 1.0, 2, 6000.0);
            case 'SIMBOT':
                return new FlywheelConfig(0, 0, 3.0 / 1.0, 2, 6000.0);
            default:
                return new FlywheelConfig(5, 4, 3.0 / 1.0, 2, 6000.0);
        }
    })(),

    // Launcher Flywheel Config
    launcherConfig: new FlywheelConfig(11, 0, 1.0 / 1.0, 2, 6000.0),

    // Infrared Port
    infraRedPort: 0, // DIO

    // PID Constants
    gains: (() => {
        switch (RobotConstants.robotType) {
            case 'COMPBOT':
                return new Gains(0.0001, 0.0, 0.0, 0.12, 0.00635, 0);
            case 'DEVBOT':
                return new Gains(0.0001, 0.0, 0.0, 0.12, 0.00635, 0);
            case 'SIMBOT':
                return new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0);
            default:
                return new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0);
        }
    })(),
};

// Helper Classes
export class Gains {
    constructor(
        public kP: number,
        public kI: number,
        public kD: number,
        public kS: number,
        public kV: number,
        public kA: number
    ) {}
}

export class FlywheelConfig {
    constructor(
        public leftCANID: number,
        public rightCANID: number,
        public reduction: number,
        public momentOfInertia: number,
        public maxAccelerationRpmPerSec: number
    ) {}
}