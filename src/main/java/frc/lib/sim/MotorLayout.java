package frc.lib.sim;

import frc.lib.types.BiMap;

/** Singleton controlling motor association. */
public class MotorLayout {

    private static BiMap<SimulatedMotor, Integer> pdhSlot = new BiMap<>();
    private static BiMap<SimulatedMotor, Integer> rioCANLoop = new BiMap<>();
    private static BiMap<SimulatedMotor, Integer> canivoreCANLoop = new BiMap<>();

    public static void setSimulatedMotor(SimulatedMotor motor, int pdhSlot, int canSlot,
        boolean isCanivore) {
        MotorLayout.pdhSlot.insert(motor, pdhSlot);
        if (isCanivore) {
            MotorLayout.canivoreCANLoop.insert(motor, canSlot);
        } else {
            MotorLayout.rioCANLoop.insert(motor, canSlot);
        }
    }

    public static SimulatedMotor getRIOCAN(int id) {
        return rioCANLoop.getKey(id);
    }

}
