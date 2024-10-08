package frc.lib.sim;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedVortex extends SimulatedSparkMax {

    public SimulatedVortex(int canID, int pdhSlot) {
        MotorLayout.setSimulatedMotor(this, pdhSlot, canID, false);
    }

    @Override
    public boolean isFlex() {
        return true;
    }

    @Override
    public boolean isBrushless() {
        return true;
    }

    private static final DCMotor DC_MOTOR = DCMotor.getNeoVortex(1);

    @Override
    public DCMotor getMotorInfo() {
        return DC_MOTOR;
    }

}
