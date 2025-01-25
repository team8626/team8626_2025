// CoralShooterInterface.ts

export interface CoralShooterInterface {
    startShooter(newRPM: number): void;
    updateShooterRPM(newRPM: number): void;
    stopShooter(): void;

    startLauncher(newSetpoint: number): void;
    updateLauncherSetpoint(newSetpoint: number): void;
    stopLauncher(): void;

    shooterIsLoaded(): boolean;
    getShooterRPMLeft(): number;
    getShooterRPMRight(): number;
    getLauncherRPM(): number;
    getLauncherSetpoint(): number;

    /** Run flywheels at voltage */
    runCharacterizationLeft?(input: number): void;

    runCharacterizationRight?(input: number): void;

    setPID?(kP: number, kI: number, kD: number): void;

    updateInputs?(values: CoralShooterValues): void;
}

// CoralShooterValues.ts

export class CoralShooterValues {
    public launchIsEnabled: boolean = false;
    public shooterIsEnabled: boolean = false;

    public currentRPMLeft: number = 0; // RPM
    public currentRPMRight: number = 0; // RPM
    public currentRMPLauncher: number = 0; // RPM
    public currentLauncherSetpoint: number = 0; // [-1;1]

    public ampsLeft: number = 0;
    public ampsRight: number = 0;
    public ampsLauncher: number = 0;

    public isLoaded: boolean = false;
    public desiredRPM: number = CoralShooterConstants.shootRPM;

    public kP: number = 0.05;
    public kI: number = 0.0;
    public kD: number = 0.0;
    public FF: number = 0.0;
}
