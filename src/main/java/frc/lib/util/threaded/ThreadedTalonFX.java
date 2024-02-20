package frc.lib.util.threaded;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

public class ThreadedTalonFX {

    @AutoLog
    public static class TalonFXInputs {
        public long updateNumber;
        public boolean controlApplied;
        public boolean positionApplied;
        public double deviceAcceleration;
        public double deviceAncillaryDeviceTemp;
        public double deviceClosedLoopDerivativeOutput;
        public double deviceClosedLoopError;
        public double deviceClosedLoopFeedForward;
        public double deviceClosedLoopIntegratedOutput;
        public double deviceClosedLoopOutput;
        public double deviceClosedLoopProportionalOutput;
        public double deviceClosedLoopReference;
        public double deviceClosedLoopReferenceSlope;
        public double deviceDeviceTemp;
        public double deviceDifferentialAveragePosition;
        public double deviceDifferentialAverageVelocity;
        public double deviceDifferentialClosedLoopDerivativeOutput;
        public double deviceDifferentialClosedLoopError;
        public double deviceDifferentialClosedLoopFeedForward;
        public double deviceDifferentialClosedLoopIntegratedOutput;
        public double deviceDifferentialClosedLoopOutput;
        public double deviceDifferentialClosedLoopProportionalOutput;
        public double deviceDifferentialClosedLoopReference;
        public double deviceDifferentialClosedLoopReferenceSlope;
        public double deviceDifferentialDifferencePosition;
        public double deviceDifferentialDifferenceVelocity;
        public double deviceDifferentialOutput;
        public double deviceDutyCycle;
        public double deviceMotorVoltage;
        public double devicePosition;
        public double deviceProcessorTemp;
        public double deviceRotorPosition;
        public double deviceRotorVelocity;
        public double deviceStatorCurrent;
        public double deviceSupplyCurrent;
        public double deviceSupplyVoltage;
        public double deviceTorqueCurrent;
        public double deviceVelocity;
        public boolean deviceFault_BootDuringEnable;
        public boolean deviceFault_BridgeBrownout;
        public boolean deviceFault_DeviceTemp;
        public boolean deviceFault_ForwardHardLimit;
        public boolean deviceFault_ForwardSoftLimit;
        public boolean deviceFault_FusedSensorOutOfSync;
        public boolean deviceFault_Hardware;
        public boolean deviceFault_MissingDifferentialFX;
        public boolean deviceFault_OverSupplyV;
        public boolean deviceFault_ProcTemp;
        public boolean deviceFault_RemoteSensorDataInvalid;
        public boolean deviceFault_RemoteSensorPosOverflow;
        public boolean deviceFault_RemoteSensorReset;
        public boolean deviceFault_ReverseHardLimit;
        public boolean deviceFault_ReverseSoftLimit;
        public boolean deviceFault_StatorCurrLimit;
        public boolean deviceFault_SupplyCurrLimit;
        public boolean deviceFault_Undervoltage;
        public boolean deviceFault_UnlicensedFeatureInUse;
        public boolean deviceFault_UnstableSupplyV;
        public boolean deviceFault_UsingFusedCANcoderWhileUnlicensed;
        public boolean deviceIsProLicensed;
        public boolean deviceStickyFault_BootDuringEnable;
        public boolean deviceStickyFault_BridgeBrownout;
        public boolean deviceStickyFault_DeviceTemp;
        public boolean deviceStickyFault_ForwardHardLimit;
        public boolean deviceStickyFault_ForwardSoftLimit;
        public boolean deviceStickyFault_FusedSensorOutOfSync;
        public boolean deviceStickyFault_Hardware;
        public boolean deviceStickyFault_MissingDifferentialFX;
        public boolean deviceStickyFault_OverSupplyV;
        public boolean deviceStickyFault_ProcTemp;
        public boolean deviceStickyFault_RemoteSensorDataInvalid;
        public boolean deviceStickyFault_RemoteSensorPosOverflow;
        public boolean deviceStickyFault_RemoteSensorReset;
        public boolean deviceStickyFault_ReverseHardLimit;
        public boolean deviceStickyFault_ReverseSoftLimit;
        public boolean deviceStickyFault_StatorCurrLimit;
        public boolean deviceStickyFault_SupplyCurrLimit;
        public boolean deviceStickyFault_Undervoltage;
        public boolean deviceStickyFault_UnlicensedFeatureInUse;
        public boolean deviceStickyFault_UnstableSupplyV;
        public boolean deviceStickyFault_UsingFusedCANcoderWhileUnlicensed;

    }

    private TalonFX realTalon;

    /**
     * Constructs a new Talon FX motor controller object.
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
    public ThreadedTalonFX(int deviceId) {
        this(deviceId, "");
    }

    /**
     * Constructs a new Talon FX motor controller object.
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
    public ThreadedTalonFX(int deviceId, String canbus) {
        this.realTalon = new TalonFX(deviceId, canbus);
    }

    private TalonFXInputsAutoLogged bufferA = new TalonFXInputsAutoLogged();
    private TalonFXInputsAutoLogged bufferB = new TalonFXInputsAutoLogged();
    private boolean readingBufferA;
    private long numUpdates;

    private TalonFXInputsAutoLogged writeBuffer() {
        if (readingBufferA) {
            return bufferB;
        } else {
            return bufferA;
        }
    }

    private TalonFXInputsAutoLogged readBuffer() {
        if (readingBufferA) {
            return bufferA;
        } else {
            return bufferB;
        }
    }

    /** Access inner TalonFX. Should only be used for configuration or to clear faults. */
    public TalonFX getInner() {
        return realTalon;
    }

    private ControlRequest nextRequest = null;

    /** Enqueue control for next update loop. */
    public void setControl(ControlRequest req) {
        this.nextRequest = req;
    }

    private Double nextPosition = null;

    /** Enqueue position for next update loop. */
    public void setPosition(double newValue) {
        this.nextPosition = newValue;
    }

    public void update() {
        TalonFXInputsAutoLogged write = writeBuffer();
        write.updateNumber = numUpdates;

        ControlRequest req = null;
        synchronized (this) {
            if (this.nextRequest != null) {
                req = this.nextRequest;
                this.nextRequest = null;
            }
        }
        if (req != null) {
            realTalon.setControl(req);
            write.controlApplied = true;
        } else {
            write.controlApplied = false;
        }

        Double pos = null;
        synchronized (this) {
            if (this.nextPosition != null) {
                pos = this.nextPosition.doubleValue();
                this.nextPosition = null;
            }
        }
        if (pos != null) {
            realTalon.setPosition(pos);
            write.positionApplied = true;
        } else {
            write.positionApplied = false;
        }

        if (this.trackingAcceleration) {
            write.deviceAcceleration = realTalon.getAcceleration().getValue();
        }
        if (this.trackingAncillaryDeviceTemp) {
            write.deviceAncillaryDeviceTemp = realTalon.getAncillaryDeviceTemp().getValue();
        }
        if (this.trackingClosedLoopDerivativeOutput) {
            write.deviceClosedLoopDerivativeOutput =
                realTalon.getClosedLoopDerivativeOutput().getValue();
        }
        if (this.trackingClosedLoopError) {
            write.deviceClosedLoopError = realTalon.getClosedLoopError().getValue();
        }
        if (this.trackingClosedLoopFeedForward) {
            write.deviceClosedLoopFeedForward = realTalon.getClosedLoopFeedForward().getValue();
        }
        if (this.trackingClosedLoopIntegratedOutput) {
            write.deviceClosedLoopIntegratedOutput =
                realTalon.getClosedLoopIntegratedOutput().getValue();
        }
        if (this.trackingClosedLoopOutput) {
            write.deviceClosedLoopOutput = realTalon.getClosedLoopOutput().getValue();
        }
        if (this.trackingClosedLoopProportionalOutput) {
            write.deviceClosedLoopProportionalOutput =
                realTalon.getClosedLoopProportionalOutput().getValue();
        }
        if (this.trackingClosedLoopReference) {
            write.deviceClosedLoopReference = realTalon.getClosedLoopReference().getValue();
        }
        if (this.trackingClosedLoopReferenceSlope) {
            write.deviceClosedLoopReferenceSlope =
                realTalon.getClosedLoopReferenceSlope().getValue();
        }
        if (this.trackingDeviceTemp) {
            write.deviceDeviceTemp = realTalon.getDeviceTemp().getValue();
        }
        if (this.trackingDifferentialAveragePosition) {
            write.deviceDifferentialAveragePosition =
                realTalon.getDifferentialAveragePosition().getValue();
        }
        if (this.trackingDifferentialAverageVelocity) {
            write.deviceDifferentialAverageVelocity =
                realTalon.getDifferentialAverageVelocity().getValue();
        }
        if (this.trackingDifferentialClosedLoopDerivativeOutput) {
            write.deviceDifferentialClosedLoopDerivativeOutput =
                realTalon.getDifferentialClosedLoopDerivativeOutput().getValue();
        }
        if (this.trackingDifferentialClosedLoopError) {
            write.deviceDifferentialClosedLoopError =
                realTalon.getDifferentialClosedLoopError().getValue();
        }
        if (this.trackingDifferentialClosedLoopFeedForward) {
            write.deviceDifferentialClosedLoopFeedForward =
                realTalon.getDifferentialClosedLoopFeedForward().getValue();
        }
        if (this.trackingDifferentialClosedLoopIntegratedOutput) {
            write.deviceDifferentialClosedLoopIntegratedOutput =
                realTalon.getDifferentialClosedLoopIntegratedOutput().getValue();
        }
        if (this.trackingDifferentialClosedLoopOutput) {
            write.deviceDifferentialClosedLoopOutput =
                realTalon.getDifferentialClosedLoopOutput().getValue();
        }
        if (this.trackingDifferentialClosedLoopProportionalOutput) {
            write.deviceDifferentialClosedLoopProportionalOutput =
                realTalon.getDifferentialClosedLoopProportionalOutput().getValue();
        }
        if (this.trackingDifferentialClosedLoopReference) {
            write.deviceDifferentialClosedLoopReference =
                realTalon.getDifferentialClosedLoopReference().getValue();
        }
        if (this.trackingDifferentialClosedLoopReferenceSlope) {
            write.deviceDifferentialClosedLoopReferenceSlope =
                realTalon.getDifferentialClosedLoopReferenceSlope().getValue();
        }
        if (this.trackingDifferentialDifferencePosition) {
            write.deviceDifferentialDifferencePosition =
                realTalon.getDifferentialDifferencePosition().getValue();
        }
        if (this.trackingDifferentialDifferenceVelocity) {
            write.deviceDifferentialDifferenceVelocity =
                realTalon.getDifferentialDifferenceVelocity().getValue();
        }
        if (this.trackingDifferentialOutput) {
            write.deviceDifferentialOutput = realTalon.getDifferentialOutput().getValue();
        }
        if (this.trackingDutyCycle) {
            write.deviceDutyCycle = realTalon.getDutyCycle().getValue();
        }
        if (this.trackingMotorVoltage) {
            write.deviceMotorVoltage = realTalon.getMotorVoltage().getValue();
        }
        if (this.trackingPosition) {
            write.devicePosition = realTalon.getPosition().getValue();
        }
        if (this.trackingProcessorTemp) {
            write.deviceProcessorTemp = realTalon.getProcessorTemp().getValue();
        }
        if (this.trackingRotorPosition) {
            write.deviceRotorPosition = realTalon.getRotorPosition().getValue();
        }
        if (this.trackingRotorVelocity) {
            write.deviceRotorVelocity = realTalon.getRotorVelocity().getValue();
        }
        if (this.trackingStatorCurrent) {
            write.deviceStatorCurrent = realTalon.getStatorCurrent().getValue();
        }
        if (this.trackingSupplyCurrent) {
            write.deviceSupplyCurrent = realTalon.getSupplyCurrent().getValue();
        }
        if (this.trackingSupplyVoltage) {
            write.deviceSupplyVoltage = realTalon.getSupplyVoltage().getValue();
        }
        if (this.trackingTorqueCurrent) {
            write.deviceTorqueCurrent = realTalon.getTorqueCurrent().getValue();
        }
        if (this.trackingVelocity) {
            write.deviceVelocity = realTalon.getVelocity().getValue();
        }

        if (this.trackingFault_BootDuringEnable) {
            write.deviceFault_BootDuringEnable = realTalon.getFault_BootDuringEnable().getValue();
        }
        if (this.trackingFault_BridgeBrownout) {
            write.deviceFault_BridgeBrownout = realTalon.getFault_BridgeBrownout().getValue();
        }
        if (this.trackingFault_DeviceTemp) {
            write.deviceFault_DeviceTemp = realTalon.getFault_DeviceTemp().getValue();
        }
        if (this.trackingFault_ForwardHardLimit) {
            write.deviceFault_ForwardHardLimit = realTalon.getFault_ForwardHardLimit().getValue();
        }
        if (this.trackingFault_ForwardSoftLimit) {
            write.deviceFault_ForwardSoftLimit = realTalon.getFault_ForwardSoftLimit().getValue();
        }
        if (this.trackingFault_FusedSensorOutOfSync) {
            write.deviceFault_FusedSensorOutOfSync =
                realTalon.getFault_FusedSensorOutOfSync().getValue();
        }
        if (this.trackingFault_Hardware) {
            write.deviceFault_Hardware = realTalon.getFault_Hardware().getValue();
        }
        if (this.trackingFault_MissingDifferentialFX) {
            write.deviceFault_MissingDifferentialFX =
                realTalon.getFault_MissingDifferentialFX().getValue();
        }
        if (this.trackingFault_OverSupplyV) {
            write.deviceFault_OverSupplyV = realTalon.getFault_OverSupplyV().getValue();
        }
        if (this.trackingFault_ProcTemp) {
            write.deviceFault_ProcTemp = realTalon.getFault_ProcTemp().getValue();
        }
        if (this.trackingFault_RemoteSensorDataInvalid) {
            write.deviceFault_RemoteSensorDataInvalid =
                realTalon.getFault_RemoteSensorDataInvalid().getValue();
        }
        if (this.trackingFault_RemoteSensorPosOverflow) {
            write.deviceFault_RemoteSensorPosOverflow =
                realTalon.getFault_RemoteSensorPosOverflow().getValue();
        }
        if (this.trackingFault_RemoteSensorReset) {
            write.deviceFault_RemoteSensorReset = realTalon.getFault_RemoteSensorReset().getValue();
        }
        if (this.trackingFault_ReverseHardLimit) {
            write.deviceFault_ReverseHardLimit = realTalon.getFault_ReverseHardLimit().getValue();
        }
        if (this.trackingFault_ReverseSoftLimit) {
            write.deviceFault_ReverseSoftLimit = realTalon.getFault_ReverseSoftLimit().getValue();
        }
        if (this.trackingFault_StatorCurrLimit) {
            write.deviceFault_StatorCurrLimit = realTalon.getFault_StatorCurrLimit().getValue();
        }
        if (this.trackingFault_SupplyCurrLimit) {
            write.deviceFault_SupplyCurrLimit = realTalon.getFault_SupplyCurrLimit().getValue();
        }
        if (this.trackingFault_Undervoltage) {
            write.deviceFault_Undervoltage = realTalon.getFault_Undervoltage().getValue();
        }
        if (this.trackingFault_UnlicensedFeatureInUse) {
            write.deviceFault_UnlicensedFeatureInUse =
                realTalon.getFault_UnlicensedFeatureInUse().getValue();
        }
        if (this.trackingFault_UnstableSupplyV) {
            write.deviceFault_UnstableSupplyV = realTalon.getFault_UnstableSupplyV().getValue();
        }
        if (this.trackingFault_UsingFusedCANcoderWhileUnlicensed) {
            write.deviceFault_UsingFusedCANcoderWhileUnlicensed =
                realTalon.getFault_UsingFusedCANcoderWhileUnlicensed().getValue();
        }
        if (this.trackingIsProLicensed) {
            write.deviceIsProLicensed = realTalon.getIsProLicensed().getValue();
        }
        if (this.trackingStickyFault_BootDuringEnable) {
            write.deviceStickyFault_BootDuringEnable =
                realTalon.getStickyFault_BootDuringEnable().getValue();
        }
        if (this.trackingStickyFault_BridgeBrownout) {
            write.deviceStickyFault_BridgeBrownout =
                realTalon.getStickyFault_BridgeBrownout().getValue();
        }
        if (this.trackingStickyFault_DeviceTemp) {
            write.deviceStickyFault_DeviceTemp = realTalon.getStickyFault_DeviceTemp().getValue();
        }
        if (this.trackingStickyFault_ForwardHardLimit) {
            write.deviceStickyFault_ForwardHardLimit =
                realTalon.getStickyFault_ForwardHardLimit().getValue();
        }
        if (this.trackingStickyFault_ForwardSoftLimit) {
            write.deviceStickyFault_ForwardSoftLimit =
                realTalon.getStickyFault_ForwardSoftLimit().getValue();
        }
        if (this.trackingStickyFault_FusedSensorOutOfSync) {
            write.deviceStickyFault_FusedSensorOutOfSync =
                realTalon.getStickyFault_FusedSensorOutOfSync().getValue();
        }
        if (this.trackingStickyFault_Hardware) {
            write.deviceStickyFault_Hardware = realTalon.getStickyFault_Hardware().getValue();
        }
        if (this.trackingStickyFault_MissingDifferentialFX) {
            write.deviceStickyFault_MissingDifferentialFX =
                realTalon.getStickyFault_MissingDifferentialFX().getValue();
        }
        if (this.trackingStickyFault_OverSupplyV) {
            write.deviceStickyFault_OverSupplyV = realTalon.getStickyFault_OverSupplyV().getValue();
        }
        if (this.trackingStickyFault_ProcTemp) {
            write.deviceStickyFault_ProcTemp = realTalon.getStickyFault_ProcTemp().getValue();
        }
        if (this.trackingStickyFault_RemoteSensorDataInvalid) {
            write.deviceStickyFault_RemoteSensorDataInvalid =
                realTalon.getStickyFault_RemoteSensorDataInvalid().getValue();
        }
        if (this.trackingStickyFault_RemoteSensorPosOverflow) {
            write.deviceStickyFault_RemoteSensorPosOverflow =
                realTalon.getStickyFault_RemoteSensorPosOverflow().getValue();
        }
        if (this.trackingStickyFault_RemoteSensorReset) {
            write.deviceStickyFault_RemoteSensorReset =
                realTalon.getStickyFault_RemoteSensorReset().getValue();
        }
        if (this.trackingStickyFault_ReverseHardLimit) {
            write.deviceStickyFault_ReverseHardLimit =
                realTalon.getStickyFault_ReverseHardLimit().getValue();
        }
        if (this.trackingStickyFault_ReverseSoftLimit) {
            write.deviceStickyFault_ReverseSoftLimit =
                realTalon.getStickyFault_ReverseSoftLimit().getValue();
        }
        if (this.trackingStickyFault_StatorCurrLimit) {
            write.deviceStickyFault_StatorCurrLimit =
                realTalon.getStickyFault_StatorCurrLimit().getValue();
        }
        if (this.trackingStickyFault_SupplyCurrLimit) {
            write.deviceStickyFault_SupplyCurrLimit =
                realTalon.getStickyFault_SupplyCurrLimit().getValue();
        }
        if (this.trackingStickyFault_Undervoltage) {
            write.deviceStickyFault_Undervoltage =
                realTalon.getStickyFault_Undervoltage().getValue();
        }
        if (this.trackingStickyFault_UnlicensedFeatureInUse) {
            write.deviceStickyFault_UnlicensedFeatureInUse =
                realTalon.getStickyFault_UnlicensedFeatureInUse().getValue();
        }
        if (this.trackingStickyFault_UnstableSupplyV) {
            write.deviceStickyFault_UnstableSupplyV =
                realTalon.getStickyFault_UnstableSupplyV().getValue();
        }
        if (this.trackingStickyFault_UsingFusedCANcoderWhileUnlicensed) {
            write.deviceStickyFault_UsingFusedCANcoderWhileUnlicensed =
                realTalon.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue();
        }


        numUpdates++;
        synchronized (this) {
            readingBufferA = !readingBufferA;
        }
    }

    public synchronized TalonFXInputsAutoLogged get() {
        return readBuffer().clone();
    }

    public boolean trackingAcceleration;
    public boolean trackingAncillaryDeviceTemp;
    public boolean trackingClosedLoopDerivativeOutput;
    public boolean trackingClosedLoopError;
    public boolean trackingClosedLoopFeedForward;
    public boolean trackingClosedLoopIntegratedOutput;
    public boolean trackingClosedLoopOutput;
    public boolean trackingClosedLoopProportionalOutput;
    public boolean trackingClosedLoopReference;
    public boolean trackingClosedLoopReferenceSlope;
    public boolean trackingDeviceTemp;
    public boolean trackingDifferentialAveragePosition;
    public boolean trackingDifferentialAverageVelocity;
    public boolean trackingDifferentialClosedLoopDerivativeOutput;
    public boolean trackingDifferentialClosedLoopError;
    public boolean trackingDifferentialClosedLoopFeedForward;
    public boolean trackingDifferentialClosedLoopIntegratedOutput;
    public boolean trackingDifferentialClosedLoopOutput;
    public boolean trackingDifferentialClosedLoopProportionalOutput;
    public boolean trackingDifferentialClosedLoopReference;
    public boolean trackingDifferentialClosedLoopReferenceSlope;
    public boolean trackingDifferentialDifferencePosition;
    public boolean trackingDifferentialDifferenceVelocity;
    public boolean trackingDifferentialOutput;
    public boolean trackingDutyCycle;
    public boolean trackingMotorVoltage;
    public boolean trackingPosition;
    public boolean trackingProcessorTemp;
    public boolean trackingRotorPosition;
    public boolean trackingRotorVelocity;
    public boolean trackingStatorCurrent;
    public boolean trackingSupplyCurrent;
    public boolean trackingSupplyVoltage;
    public boolean trackingTorqueCurrent;
    public boolean trackingVelocity;
    public boolean trackingFault_BootDuringEnable;
    public boolean trackingFault_BridgeBrownout;
    public boolean trackingFault_DeviceTemp;
    public boolean trackingFault_ForwardHardLimit;
    public boolean trackingFault_ForwardSoftLimit;
    public boolean trackingFault_FusedSensorOutOfSync;
    public boolean trackingFault_Hardware;
    public boolean trackingFault_MissingDifferentialFX;
    public boolean trackingFault_OverSupplyV;
    public boolean trackingFault_ProcTemp;
    public boolean trackingFault_RemoteSensorDataInvalid;
    public boolean trackingFault_RemoteSensorPosOverflow;
    public boolean trackingFault_RemoteSensorReset;
    public boolean trackingFault_ReverseHardLimit;
    public boolean trackingFault_ReverseSoftLimit;
    public boolean trackingFault_StatorCurrLimit;
    public boolean trackingFault_SupplyCurrLimit;
    public boolean trackingFault_Undervoltage;
    public boolean trackingFault_UnlicensedFeatureInUse;
    public boolean trackingFault_UnstableSupplyV;
    public boolean trackingFault_UsingFusedCANcoderWhileUnlicensed;
    public boolean trackingIsProLicensed;
    public boolean trackingStickyFault_BootDuringEnable;
    public boolean trackingStickyFault_BridgeBrownout;
    public boolean trackingStickyFault_DeviceTemp;
    public boolean trackingStickyFault_ForwardHardLimit;
    public boolean trackingStickyFault_ForwardSoftLimit;
    public boolean trackingStickyFault_FusedSensorOutOfSync;
    public boolean trackingStickyFault_Hardware;
    public boolean trackingStickyFault_MissingDifferentialFX;
    public boolean trackingStickyFault_OverSupplyV;
    public boolean trackingStickyFault_ProcTemp;
    public boolean trackingStickyFault_RemoteSensorDataInvalid;
    public boolean trackingStickyFault_RemoteSensorPosOverflow;
    public boolean trackingStickyFault_RemoteSensorReset;
    public boolean trackingStickyFault_ReverseHardLimit;
    public boolean trackingStickyFault_ReverseSoftLimit;
    public boolean trackingStickyFault_StatorCurrLimit;
    public boolean trackingStickyFault_SupplyCurrLimit;
    public boolean trackingStickyFault_Undervoltage;
    public boolean trackingStickyFault_UnlicensedFeatureInUse;
    public boolean trackingStickyFault_UnstableSupplyV;
    public boolean trackingStickyFault_UsingFusedCANcoderWhileUnlicensed;
}
