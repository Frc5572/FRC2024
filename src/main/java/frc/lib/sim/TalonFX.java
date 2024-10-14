package frc.lib.sim;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class TalonFX implements MotorController, Sendable, AutoCloseable {

    public TalonFX(int deviceId, String canbus) {

    }

    public TalonFX(int deviceId) {
        this(deviceId, "*");
    }

    @Override
    public void close() throws Exception {
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
    }

    @Override
    public void set(double speed) {
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public double get() {
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }

    @Override
    public void setInverted(boolean isInverted) {
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public boolean getInverted() {
        throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
    }

    @Override
    public void disable() {
        throw new UnsupportedOperationException("Unimplemented method 'disable'");
    }

    @Override
    public void stopMotor() {
        throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
    }

}
