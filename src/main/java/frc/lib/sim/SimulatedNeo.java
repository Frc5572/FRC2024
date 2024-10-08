package frc.lib.sim;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedNeo extends SimulatedSparkMax {

    public SimulatedNeo(int canID, int pdhSlot) {
        MotorLayout.setSimulatedMotor(this, pdhSlot, canID, false);
    }

    @Override
    public boolean isFlex() {
        return false;
    }

    @Override
    public boolean isBrushless() {
        return true;
    }

    private static final DCMotor DC_MOTOR = DCMotor.getNEO(1);

    @Override
    public DCMotor getMotorInfo() {
        return DC_MOTOR;
    }

}
