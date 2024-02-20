package frc.lib.util.threaded;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.hardware.CANcoder;

public class ThreadedCanCoder {

    @AutoLog
    public static class CANCoderInputs {
        public long updateNumber;
    }

    private CANcoder realCanCoder;

    /**
     * Constructs a new CANcoder object.
     * <p>
     * Constructs the device using the default CAN bus for the system:
     * <ul>
     * <li>"rio" on roboRIO
     * <li>"can0" on Linux
     * <li>"*" on Windows
     * </ul>
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     */
    public ThreadedCanCoder(int deviceId) {
        this(deviceId, "");
    }

    /**
     * Constructs a new CANcoder object.
     *
     * Possible CAN bus strings are:
     * <ul>
     * <li>"rio" for the native roboRIO CAN bus
     * <li>CANivore name or serial number
     * <li>SocketCAN interface (non-FRC Linux only)
     * <li>"*" for any CANivore seen by the program
     * <li>empty string (default) to select the default for the system:
     * <ul>
     * <li>"rio" on roboRIO
     * <li>"can0" on Linux
     * <li>"*" on Windows
     * </ul>
     * </ul>
     * 
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param canbus Name of the CAN bus this device is on.
     */
    public ThreadedCanCoder(int deviceId, String canbus) {
        this.realCanCoder = new CANcoder(deviceId, canbus);
    }

    private CANCoderInputsAutoLogged bufferA = new CANCoderInputsAutoLogged();
    private CANCoderInputsAutoLogged bufferB = new CANCoderInputsAutoLogged();
    private boolean readingBufferA;
    private long numUpdates;

    private CANCoderInputsAutoLogged writeBuffer() {
        if (readingBufferA) {
            return bufferB;
        } else {
            return bufferA;
        }
    }

    private CANCoderInputsAutoLogged readBuffer() {
        if (readingBufferA) {
            return bufferA;
        } else {
            return bufferB;
        }
    }

    /** Access inner CANCoder. Should only be used for configuration or to clear faults. */
    public CANcoder getInner() {
        return realCanCoder;
    }

    public void update() {
        CANCoderInputsAutoLogged write = writeBuffer();
        write.updateNumber = numUpdates;

        numUpdates++;
        synchronized (this) {
            readingBufferA = !readingBufferA;
        }
    }

    public synchronized CANCoderInputsAutoLogged get() {
        return readBuffer().clone();
    }

}
