//Fix
package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.encoder.GenericEncoder;
import team7111.robot.utils.encoder.ThroughBore;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase {
    
    public enum ShooterState {
        prepareShot,
        shoot,
        reverse,
        prepareShotVision,
        defaultState,
        manual,
        idle,
    };

    private ShooterState state = ShooterState.defaultState;
    
    private VisionSubsystem vision;

    private double visionAngle = 0;
    private double visionSpeed = 0.5;
    private double previousAngle = 0;
    private double manualPivotSetpoint = 0;

    private double currentSetpoint = 37;

    private SparkMax flywheelMotor = new SparkMax(10, MotorType.kBrushless);
    private SparkMax flywheelFollowerMotor = new SparkMax(12, MotorType.kBrushless);

    private GenericEncoder pivotEncoder = new ThroughBore(1, 2, 1/(8192*17.5) * 4);

    //private PositionDutyCycle posDutyCycle = new PositionDutyCycle(37);

    private SmartMotorControllerConfig pivotControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(7, 0.0, 0)
        .withIdleMode(MotorMode.COAST)
        //.withSoftLimit(Degree.of(0), Degree.of(90))
        .withMotorInverted(true)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withTelemetry("shooterPivotMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(48/12, 24/15, 210/12)))
        .withStartingPosition(Degrees.of(37))
        //.withExternalEncoder(new Encoder(4, 5))
        //.withExternalEncoderGearing(210/12)
        //.withExternalEncoderInverted(false)
        //.withExternalEncoderZeroOffset(Degrees.of(0))
        ;

    private TalonFX pivotMotor = new TalonFX(15);
    private SmartMotorController pivotController = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotControllerConfig);
    
    private PivotConfig pivotConfig = new PivotConfig(pivotController)
        .withStartingPosition(Degrees.of(37))
        .withMOI(Inches.of(15.5), Pounds.of(3.953))
        .withHardLimit(Degrees.of(37), Degrees.of(67))
        .withTelemetry("ShooterPivot", TelemetryVerbosity.HIGH);

    private Pivot pivot = new Pivot(pivotConfig);

    

    private SparkBaseConfig sparkBaseConfigFollower = new SparkMaxConfig()
        .follow(10, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);
        
    private SparkBaseConfig sparkBaseConfigLead = new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(40);
        
    

    public ShooterSubsystem(VisionSubsystem vision) {
        this.vision = vision;
        flywheelMotor.configure(sparkBaseConfigLead, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        flywheelFollowerMotor.configure(sparkBaseConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets motor positions based off of state.
     * <p>
     * 
     */
    public void periodic() {
        if(RobotBase.isReal()) {
            pivotMotor.setPosition(Degrees.of(-pivotEncoder.getPosition().getDegrees() + 37).in(Rotations));
        }
        pivot.updateTelemetry();
        visionAngle = vision.shooterAngle();
        manageState();

        SmartDashboard.putNumber("throughbore position", -pivotEncoder.getPosition().getDegrees());
        SmartDashboard.putNumber("pivot position", Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Pivot Velocity", pivotMotor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Pivot Setpoint", currentSetpoint);
        SmartDashboard.putNumber("Vision Shooter Setpoint", visionAngle);
    }

    public void simulationPeriodic(){
        pivot.simIterate();
    }

    public void setManualSpeed(double speed){
        flywheelMotor.set(speed);
    }

    public void addManualAngle(double angleIncrement){
        manualPivotSetpoint += angleIncrement;
        if(manualPivotSetpoint > 67)
            manualPivotSetpoint = 67;
        if(manualPivotSetpoint < 37)
            manualPivotSetpoint = 37;
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(manualPivotSetpoint).in(Rotations)));
        
    }

    /**
     * Sets the shooter angle from minimum to maximum shooter angle.
     * Minimum angle is 
     */
    public void setAngle(double angle){
        visionAngle = angle;
    }

    private void manageState() {
        switch (state) {
            case prepareShot:
                prepareShot();
                break;
            case shoot:
                shoot();
                break;
            case reverse:
                reverse();
                break;
            case prepareShotVision:
                prepareShotVision();
                break;
            case manual:
                manual();
                break;
            case idle:
                idle();
                break;
            case defaultState:
                defaultState();
                break;
        }
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    private void idle() {

        flywheelMotor.set(0);
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(37).in(Rotations)));
        manualPivotSetpoint = 0;
        currentSetpoint = 37;
    }

    private void prepareShot() {
        // placeholder values. pivot will be extended and wheels will rev-up
        flywheelMotor.set(0.8);
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(67).in(Rotations)));
        previousAngle = 67;
        manualPivotSetpoint = 67;
        currentSetpoint = 67;
    }

    private void shoot() {
        // placeholder values. wheels will be shooting
        flywheelMotor.set(0.8);
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(previousAngle).in(Rotations)));
        manualPivotSetpoint = previousAngle;
    }

    private void reverse() {
        // placeholder values. wheels will be reversed
        flywheelMotor.set(-0.25);
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(65).in(Rotations)));
        manualPivotSetpoint = 65;
    }

    private void prepareShotVision() {
        // set shooterPivot to visionAngle and shooter to visionSpeed
        pivotMotor.setControl(new PositionDutyCycle(Degrees.of(visionAngle).in(Rotations)));
        flywheelMotor.set(visionSpeed);
        previousAngle = visionAngle;
        manualPivotSetpoint = previousAngle;
    }

    private void defaultState() {} 

    private void manual() {}
}
