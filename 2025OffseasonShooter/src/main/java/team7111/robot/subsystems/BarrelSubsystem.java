//Fix
package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class BarrelSubsystem extends SubsystemBase {

    public enum BarrelState {
        intake,
        adjust,
        readjust,
        shoot,
        reverse,
        unload,
        loaded,
        defaultState,
        manual,
    };

    //Dutycycle is on a scale of -1 (full reverse) to 1 (full forward)
    private final double intakeDutycycle = 0.3;
    private final double adjustDutycycle = -0.1;
    private final double readjustDutycycle = -0.4;
    private final double shootDutycycle = 0.5;
    private final double reverseDutycycle = -0.3;
    private final double stopped = 0;

    private DigitalInput beamBreak = new DigitalInput(8);

    private SparkMax barrelMotor = new SparkMax(14, MotorType.kBrushless);
    private SmartMotorControllerConfig barrelMotorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(new PIDController(0.1, 0, 0))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(2.5, 1)))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("BarrelMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(true)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new SimpleMotorFeedforward(0, 0))
        .withControlMode(ControlMode.OPEN_LOOP);
    
    private SmartMotorController barrelController = new SparkWrapper(barrelMotor, DCMotor.getNEO(1), barrelMotorConfig);
    private FlyWheelConfig barrelConfig = new FlyWheelConfig(barrelController)
        .withDiameter(Inches.of(1))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(1000))
        .withTelemetry("Barrel", TelemetryVerbosity.HIGH);

    private FlyWheel barrel = new FlyWheel(barrelConfig);

    private BarrelState state = BarrelState.defaultState;

    //Constructor for class
    public BarrelSubsystem() {}

    /**
     * Updates the barrel telemetry
     */
    @Override
    public void periodic() {
        manageState();
        barrel.updateTelemetry();
        SmartDashboard.putBoolean("isBeamBroken", getBeamBreak());
    }

    /**
     * Runs the barrel in simulation settings.
     */
    @Override
    public void simulationPeriodic() {
        barrel.simIterate();
    }

    /**
     * Spins the motor to shoot the projectile using dutycycle.
     * @param dutycycle Value of -1 (full reverse) to 1 (full forward).
     */
    public void setManualSpeed(double dutycycle){
        barrel.set(dutycycle).execute();
    }

    /**
     * Takes action on what the current set state is.
     * @see setState -Sets the state for this method to act on.
     * @see getState -Returns the current state, useful for debugging.
     */
    private void manageState() {
        switch(state) {
            case intake:
                intake();
                break;
            case adjust:
                adjust();
                break;
            case readjust:
                readjust();
                break;
            case shoot:
                shoot();
                break;
            case reverse:
                reverse();
                break;
            case unload:
                unloaded();
                break;
            case loaded:
                loaded();
                break;
            case manual:
                manual();
                break;
            case defaultState:
                defaultState();
                break;
        }
    }

    /**
     * Returns the currently set state. Useful for debugging and printing to smart dashboard.
     */
    public BarrelState getState() {
        return state;
    }

    /**
     * Sets the current state.
     * @see manageState -Acts on the current state, this method does nothing if manageState is not called.
     */
    public void setState(BarrelState state) {
        this.state = state;
    }

    /**
     * Gets the boolean for the beambrake. True is beam broken, false is beam is not broken.
     * <p>
     * CURRENTLY HARD CODED TO FALSE
     */
    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    /**
     * Sets the barrel to the intake dutycycle. 
     */
    private void intake() {
        // Placeholder values. Wheels active
        barrel.set(intakeDutycycle).execute();
    }

    /**
     * Sets the barrel to slowly move backwards.
     */
    private void adjust() {
        //  Placeholder values. Moves wheels backward slowly
        barrel.set(adjustDutycycle).execute();
    }

    /**
     * Sets the barrel to move the wheels forward.
     */
    private void readjust() {
        //  Placeholder values. Moves wheels forward
        barrel.set(readjustDutycycle).execute();
    }

    /**
     * Sets the barrel to move the wheels at full speed forward.
     */
    private void shoot() {
        // Placeholder values. Wheels will be active at full speed
        barrel.set(shootDutycycle).execute();
    }

    /**
     * Sets the barrel to move the wheels slowly backward.
     */
    private void reverse() {
        // Placeholder values. Sets wheels reversing slowly
        barrel.set(reverseDutycycle).execute();
    }

    /**
     * Sets the barrel wheels to be stopped.
     */
    private void unloaded() {
        barrel.set(stopped).execute();
    }

    /**
     * Sets the barrel wheels to be stopped.
     */
    private void loaded() {
        // Game piece is stored, wheels inactive
        barrel.set(stopped).execute();
    }

    /**
     * Manual code for the barrel.
     */
    private void manual() {

    }

    /**
     * Sets the barrel to the default state of stopped.
     */
    private void defaultState() {
        //  Wheels inactive
        barrel.set(stopped).execute();
    }
}
