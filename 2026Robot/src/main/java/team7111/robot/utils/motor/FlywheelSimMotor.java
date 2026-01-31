package team7111.robot.utils.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team7111.robot.utils.encoder.GenericEncoder;

public class FlywheelSimMotor implements Motor{

    public FlywheelSimMotor(GenericEncoder encoder, FlywheelSim motor, PIDController pid, SimpleMotorFeedforward ff){

    }

    @Override
    public void setDutyCycle(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDutyCycle'");
    }

    @Override
    public double getDutyCycle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDutyCycle'");
    }

    @Override
    public void setVelocity(double rpm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public void setPositionReadout(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionReadout'");
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setSetpoint(double setPoint, boolean useSimFF) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
    }

    @Override
    public void setPID(double p, double i, double d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPID'");
    }

    @Override
    public PIDController getPID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPID'");
    }

    @Override
    public void setSpeedLimits(double positiveSpeed, double negativeSpeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeedLimits'");
    }

    @Override
    public GenericEncoder getEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
    }

    @Override
    public double getVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
    }

    @Override
    public void setVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public boolean isAtSetpoint(double deadzone) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isAtSetpoint'");
    }

    @Override
    public SimpleMotorFeedforward getFeedForward() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFeedForward'");
    }

    @Override
    public void setFeedFoward(double kS, double kV, double kA) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFeedFoward'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

}
