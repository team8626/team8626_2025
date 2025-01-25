// CoralShooterSubsystem.ts

import { SmartDashboard } from 'edu.wpi.first.wpilibj.smartdashboard';
import { CS_SubsystemBase } from '../CS_SubsystemBase';
import { CoralShooterInterface, CoralShooterValues } from './CoralShooterInterface';
import { CoralShooterConstants } from './CoralShooterConstants';
import { CS_Utils } from '../utils/CS_Utils';

export class CoralShooterSubsystem extends CS_SubsystemBase {
    private coralShooterInterface: CoralShooterInterface;
    private values: CoralShooterValues = new CoralShooterValues();
    private desiredRPM: number = CoralShooterConstants.shootRPM;
    private desiredLauncherSetpoint: number = CoralShooterConstants.launcherShootSetpoint;

    constructor(subsystemInterface: CoralShooterInterface) {
        super();
        this.coralShooterInterface = subsystemInterface;
        console.log('CoralShooterSubsystem initialized');
    }

    // Interface Calls
    public startRampUp(): void {
        this.coralShooterInterface.startShooter(this.desiredRPM);
    }

    public startShooter(newRPM: number): void {
        this.desiredRPM = newRPM;
        this.coralShooterInterface.startShooter(newRPM);
    }

    public setShooterRPM(newRPM: number): void {
        this.desiredRPM = newRPM;
        this.coralShooterInterface.updateShooterRPM(newRPM);
    }

    public setShootingRPM(): void {
        this.coralShooterInterface.updateShooterRPM(this.desiredRPM);
    }

    public startIntake(): void {
        this.coralShooterInterface.startShooter(CoralShooterConstants.intakeRPM);
        this.coralShooterInterface.startLauncher(CoralShooterConstants.launcherIntakeSetpoint);
    }

    public startLauncher(): void {
        this.coralShooterInterface.startLauncher(this.desiredLauncherSetpoint);
    }

    public startLauncherWithSetpoint(newSetpoint: number): void {
        this.desiredLauncherSetpoint = newSetpoint;
        this.coralShooterInterface.startLauncher(newSetpoint);
    }

    public stopShooter(): void {
        this.coralShooterInterface.stopShooter();
    }

    public stopLauncher(): void {
        this.desiredLauncherSetpoint = 0;
        this.coralShooterInterface.stopLauncher();
    }

    public stopAll(): void {
        this.coralShooterInterface.stopShooter();
        this.coralShooterInterface.stopLauncher();
    }

    public getShooterRPMLeft(): number {
        return this.coralShooterInterface.getShooterRPMLeft();
    }

    public getShooterRPMRight(): number {
        return this.coralShooterInterface.getShooterRPMRight();
    }

    public isLoaded(): boolean {
        return this.coralShooterInterface.shooterIsLoaded();
    }

    public setPID(newkP: number, newkI: number, newkD: number): void {
        this.coralShooterInterface.setPID(newkP, newkI, newkD);
    }

    public setkP(newkP: number): void {
        this.coralShooterInterface.setPID(newkP, this.values.kI, this.values.kD);
    }

    public setkI(newkI: number): void {
        this.coralShooterInterface.setPID(this.values.kP, newkI, this.values.kD);
    }

    public setkD(newkD: number): void {
        this.coralShooterInterface.setPID(this.values.kP, this.values.kI, newkD);
    }

    public CS_periodic(): void {
        this.coralShooterInterface.updateInputs(this.values);
    }

    public initDashboard(): void {
        console.log('Initializing Dashboard');
        SmartDashboard.putNumber('Subsystem/CoralShooter/Gains/P', CoralShooterConstants.gains.kP());
        SmartDashboard.putNumber('Subsystem/CoralShooter/Gains/I', CoralShooterConstants.gains.kI());
        SmartDashboard.putNumber('Subsystem/CoralShooter/Gains/D', CoralShooterConstants.gains.kD());
    }

    public updateDashboard(): void {
        const newkP = SmartDashboard.getNumber('Subsystem/CoralShooter/Gains/P', this.values.kP);
        const newkI = SmartDashboard.getNumber('Subsystem/CoralShooter/Gains/I', this.values.kI);
        const newkD = SmartDashboard.getNumber('Subsystem/CoralShooter/Gains/D', this.values.kD);

        this.values.kP = CS_Utils.updateFromSmartDashboard(newkP, this.values.kP, (value) => this.setkP(value));
        this.values.kI = CS_Utils.updateFromSmartDashboard(newkI, this.values.kI, (value) => this.setkI(value));
        this.values.kD = CS_Utils.updateFromSmartDashboard(newkD, this.values.kD, (value) => this.setkD(value));

        SmartDashboard.putBoolean('Subsystem/CoralShooter/Shooter', this.values.shooterIsEnabled);
        SmartDashboard.putBoolean('Subsystem/CoralShooter/Launcher', this.values.launchIsEnabled);

        SmartDashboard.putNumber('Subsystem/CoralShooter/Shooter RPM Left', this.values.currentRPMLeft);
        SmartDashboard.putNumber('Subsystem/CoralShooter/Shooter RPM Right', this.values.currentRPMRight);
        SmartDashboard.putNumber('Subsystem/CoralShooter/Launcher Setpoint', this.desiredLauncherSetpoint);

        SmartDashboard.putNumber('Subsystem/CoralShooter/Shooter Amps Left', this.values.ampsLeft);
        SmartDashboard.putNumber('Subsystem/CoralShooter/Shooter Amps Right', this.values.ampsRight);
        SmartDashboard.putNumber('Subsystem/CoralShooter/Launcher Amps', this.values.ampsLauncher);

        SmartDashboard.putBoolean('Subsystem/CoralShooter/isLoaded', this.values.isLoaded);

        const newRPM = SmartDashboard.getNumber('Subsystem/CoralShooter/Shooter DesiredRPM', CoralShooterConstants.shootRPM);
        if (newRPM !== this.desiredRPM) {
            this.setShooterRPM(newRPM);
        }
        SmartDashboard.putNumber('Subsystem/CoralShooter/Shooter DesiredRPM', this.desiredRPM);
    }

    public runCharacterization(input: number): void {
        this.coralShooterInterface.runCharacterizationLeft(input);
        this.coralShooterInterface.runCharacterizationRight(input);
    }

    public getCharacterizationVelocity(): number {
        return (this.values.currentRPMLeft + this.values.currentRPMRight) / 2.0;
    }
}