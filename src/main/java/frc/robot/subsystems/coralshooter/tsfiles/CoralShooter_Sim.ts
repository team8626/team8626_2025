// CoralShooter_Sim.ts

import { CoralShooterInterface, CoralShooterValues } from './CoralShooterInterface';
import { CoralShooterConstants } from './CoralShooterConstants';
import { FlywheelSim } from 'edu.wpi.first.wpilibj.simulation';
import { DIOSim } from 'edu.wpi.first.wpilibj.simulation.DIOSim';
import { DCMotor, LinearSystemId } from 'edu.wpi.first.math.system.plant';
import { Units } from 'edu.wpi.first.math.util.Units';
import { DigitalInput } from 'edu.wpi.first.wpilibj';

export class CoralShooter_Sim implements CoralShooterInterface {
    private shooterIsEnabled: boolean = false;
    private launcherIsEnabled: boolean = false;

    private currentLauncherSetpoint: number = 0;

    private leftSim: FlywheelSim;
    private rightSim: FlywheelSim;
    private launchSim: FlywheelSim;

    private loadedSensor: DigitalInput;
    private loadedSensorSim: DIOSim;

    constructor() {
        this.loadedSensor = new DigitalInput(CoralShooterConstants.infraRedPort);
        this.loadedSensorSim = new DIOSim(this.loadedSensor);

        this.leftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1),
                4 * CoralShooterConstants.flywheelConfig.momentOfInertia,
                CoralShooterConstants.flywheelConfig.reduction
            ),
            DCMotor.getNEO(1),
            0.00363458292
        );

        this.rightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1),
                4 * CoralShooterConstants.flywheelConfig.momentOfInertia,
                CoralShooterConstants.flywheelConfig.reduction
            ),
            DCMotor.getNEO(1),
            0.00363458292
        );

        this.launchSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1),
                CoralShooterConstants.flywheelConfig.momentOfInertia,
                CoralShooterConstants.flywheelConfig.reduction
            ),
            DCMotor.getNEO(1),
            0.00363458292
        );
    }

    updateInputs(values: CoralShooterValues): void {
        this.rightSim.update(0.02);
        this.leftSim.update(0.02);
        this.launchSim.update(0.02);

        values.launchIsEnabled = this.launcherIsEnabled;
        values.shooterIsEnabled = this.shooterIsEnabled;

        values.currentRPMLeft = this.getShooterRPMLeft();
        values.currentRPMRight = this.getShooterRPMRight();

        values.currentRMPLauncher = this.getLauncherRPM();
        values.currentLauncherSetpoint = this.getLauncherSetpoint();

        values.ampsLeft = this.leftSim.getCurrentDrawAmps();
        values.ampsRight = this.rightSim.getCurrentDrawAmps();
        values.ampsLauncher = this.launchSim.getCurrentDrawAmps();

        values.isLoaded = this.shooterIsLoaded();
    }

    startShooter(newRPM: number): void {
        const angularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(newRPM);
        this.rightSim.setAngularVelocity(angularVelocity);
        this.leftSim.setAngularVelocity(angularVelocity);
        this.shooterIsEnabled = true;
    }

    stopShooter(): void {
        this.updateShooterRPM(0);
        this.shooterIsEnabled = false;
    }

    updateShooterRPM(newRPM: number): void {
        if (this.shooterIsEnabled) {
            const angularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(newRPM);
            this.rightSim.setAngularVelocity(angularVelocity);
            this.leftSim.setAngularVelocity(angularVelocity);
        }
        console.log(`New Shooter RPM: ${newRPM}`);
    }

    stopLauncher(): void {
        this.updateLauncherSetpoint(0);
        this.launcherIsEnabled = false;
    }

    updateLauncherSetpoint(newSetpoint: number): void {
        this.currentLauncherSetpoint = newSetpoint;
        if (this.launcherIsEnabled) {
            const angularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(newSetpoint);
            this.launchSim.setAngularVelocity(angularVelocity);
        }
        console.log(`New Launcher Setpoint: ${newSetpoint}`);
    }

    startLauncher(newSetpoint: number): void {
        this.currentLauncherSetpoint = newSetpoint;
        this.launcherIsEnabled = true;
    }

    getShooterRPMLeft(): number {
        return this.leftSim.getAngularVelocityRPM();
    }

    getShooterRPMRight(): number {
        return this.rightSim.getAngularVelocityRPM();
    }

    getLauncherRPM(): number {
        return this.launchSim.getAngularVelocityRPM();
    }

    getLauncherSetpoint(): number {
        return this.currentLauncherSetpoint;
    }

    shooterIsLoaded(): boolean {
        return !this.loadedSensorSim.getValue();
    }

    setPID(newkP: number, newkI: number, newkD: number): void {
        console.log(`New PID: ${newkP}, ${newkI}, ${newkD}`);
    }
}