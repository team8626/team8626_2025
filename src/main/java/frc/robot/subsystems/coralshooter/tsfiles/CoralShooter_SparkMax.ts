// CoralShooter_SparkMax.ts

import { CoralShooterInterface, CoralShooterValues } from './CoralShooterInterface';
import { CoralShooterConstants } from './CoralShooterConstants';
import { SparkMax, SparkMaxConfig, MotorType, IdleMode, ControlType, ResetMode, PersistMode, ClosedLoopSlot, FeedbackSensor, ArbFFUnits } from '@revrobotics';
import { SimpleMotorFeedforward } from 'edu.wpi.first.math.controller.SimpleMotorFeedforward';
import { DigitalInput } from 'edu.wpi.first.wpilibj';

export class CoralShooter_SparkMax implements CoralShooterInterface {
    private leftMotor: SparkMax;
    private rightMotor: SparkMax;
    private launchMotor: SparkMax;

    private leftConfig: SparkMaxConfig;
    private rightConfig: SparkMaxConfig;
    private launchConfig: SparkMaxConfig;

    private leftController: any; // Adjust based on available TS libraries for FRC
    private leftEncoder: any;
    private launchController: any;

    private leftFF: SimpleMotorFeedforward;

    private loadedSensor: DigitalInput;

    private shooterIsEnabled: boolean = false;
    private launcherIsEnabled: boolean = false;
    private currentLauncherSetpoint: number = 0;

    constructor() {
        // Initialize Feedforward Controller
        this.leftFF = new SimpleMotorFeedforward(
            CoralShooterConstants.gains.kS(),
            CoralShooterConstants.gains.kV(),
            CoralShooterConstants.gains.kA()
        );

        // Initialize Motors
        this.leftConfig = new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .encoder.positionConversionFactor(1 / CoralShooterConstants.flywheelConfig.reduction)
            .velocityConversionFactor(1 / CoralShooterConstants.flywheelConfig.reduction)
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(CoralShooterConstants.gains.kP())
            .i(CoralShooterConstants.gains.kI())
            .d(CoralShooterConstants.gains.kD())
            .outputRange(-1, 1);

        this.leftMotor = new SparkMax(CoralShooterConstants.flywheelConfig.leftCANID, MotorType.kBrushless);
        this.leftMotor.configure(this.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.leftController = this.leftMotor.getClosedLoopController();
        this.leftEncoder = this.leftMotor.getEncoder();
        this.leftController.setReference(0, ControlType.kDutyCycle);

        this.rightConfig = new SparkMaxConfig().follow(this.leftMotor, true);
        this.rightMotor = new SparkMax(CoralShooterConstants.flywheelConfig.rightCANID, MotorType.kBrushless);
        this.rightMotor.configure(this.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.launchConfig = new SparkMaxConfig().inverted(false);
        this.launchMotor = new SparkMax(CoralShooterConstants.launcherConfig.leftCANID, MotorType.kBrushless);
        this.launchMotor.configure(this.launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.launchController = this.launchMotor.getClosedLoopController();
        this.launchController.setReference(0, ControlType.kDutyCycle);

        this.loadedSensor = new DigitalInput(CoralShooterConstants.infraRedPort);
    }

    updateInputs(values: CoralShooterValues): void {
        values.launchIsEnabled = this.launcherIsEnabled;
        values.shooterIsEnabled = this.shooterIsEnabled;

        values.currentRPMLeft = this.getShooterRPMLeft();
        values.currentRPMRight = this.getShooterRPMRight();

        values.currentRMPLauncher = this.getLauncherRPM();
        values.currentLauncherSetpoint = this.getLauncherSetpoint();

        values.ampsLeft = this.leftMotor.getOutputCurrent();
        values.ampsRight = this.rightMotor.getOutputCurrent();
        values.ampsLauncher = this.launchMotor.getOutputCurrent();

        values.isLoaded = this.shooterIsLoaded();
    }

    startShooter(newRPM: number): void {
        this.leftController.setReference(
            newRPM,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            this.leftFF.calculate(newRPM),
            ArbFFUnits.kVoltage
        );
        this.shooterIsEnabled = true;
    }

    stopShooter(): void {
        this.leftController.setReference(0, ControlType.kDutyCycle);
        this.shooterIsEnabled = false;
    }

    updateShooterRPM(newRPM: number): void {
        if (this.shooterIsEnabled) {
            this.leftController.setReference(
                newRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                this.leftFF.calculate(newRPM),
                ArbFFUnits.kVoltage
            );
        }
    }

    stopLauncher(): void {
        this.updateLauncherSetpoint(0);
        this.launcherIsEnabled = false;
    }

    updateLauncherSetpoint(newSetpoint: number): void {
        this.currentLauncherSetpoint = newSetpoint;
        if (this.launcherIsEnabled) {
            this.launchController.setReference(newSetpoint, ControlType.kDutyCycle);
        }
    }

    startLauncher(newSetpoint: number): void {
        this.currentLauncherSetpoint = newSetpoint;
        this.launchController.setReference(newSetpoint, ControlType.kDutyCycle);
        this.launcherIsEnabled = true;
    }

    getShooterRPMLeft(): number {
        return this.leftEncoder.getVelocity();
    }

    getShooterRPMRight(): number {
        return this.leftEncoder.getVelocity(); // Adjust if separate encoder for right
    }

    getLauncherRPM(): number {
        return this.leftEncoder.getVelocity(); // Adjust as needed
    }

    getLauncherSetpoint(): number {
        return this.currentLauncherSetpoint;
    }

    shooterIsLoaded(): boolean {
        return !this.loadedSensor.get();
    }

    setPID(newkP: number, newkI: number, newkD: number): void {
        this.leftConfig.closedLoop.p(newkP).i(newkI).d(newkD);
        this.leftMotor.configure(this.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    runCharacterizationLeft(input: number): void {
        this.leftMotor.setVoltage(input);
    }

    runCharacterizationRight(input: number): void {
        this.rightMotor.setVoltage(input);
    }
}