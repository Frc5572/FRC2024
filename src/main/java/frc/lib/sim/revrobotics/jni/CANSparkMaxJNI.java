package frc.lib.sim.revrobotics.jni;

import frc.lib.sim.MotorLayout;
import frc.lib.sim.SimulatedMotor;
import frc.lib.sim.SimulatedSparkMax;
import frc.lib.types.BiMap;

public class CANSparkMaxJNI {

    private static final BiMap<SimulatedSparkMax, Integer> SPARK_MAXES = new BiMap<>();

    public static int c_SparkMax_RegisterId(int arg0) {
        // TODO only checks for duplicate CAN id (18)
        return 0;
    }

    public static long c_SparkMax_Create(int deviceId, int typeAsInt, int model) {
        boolean isBrushless = typeAsInt != 0;
        boolean isFlex = model != 0;
        SimulatedMotor motor = MotorLayout.getRIOCAN(deviceId);
        if (motor == null) {
            throw new IllegalArgumentException("Invalid Spark " + (isFlex ? "Flex" : "Max")
                + " defined! No motor at id " + deviceId + "!");
        } else if (motor instanceof SimulatedSparkMax sparkMax) {
            if (sparkMax.isBrushless() != isBrushless) {
                String sparkMaxState = sparkMax.isBrushless() ? "Brushless" : "Brushed";
                String defState = isBrushless ? "Brushless" : "Brushed";
                throw new IllegalArgumentException(
                    "Invalid Spark " + (isFlex ? "Flex" : "Max") + " defined! In code, defined as "
                        + defState + ", but in sim defined as " + sparkMaxState + "!");
            }
            if (sparkMax.isFlex() != isFlex) {
                throw new IllegalArgumentException("Invalid Spark " + (isFlex ? "Flex" : "Max")
                    + " defined! In code, defined as Spark" + (isFlex ? "Flex" : "Max")
                    + ", but in sim defined as Spark" + (sparkMax.isFlex() ? "Flex" : "Max") + "!");
            }
            SPARK_MAXES.insert(sparkMax, System.identityHashCode(motor));
            return System.identityHashCode(motor);
        } else {
            throw new IllegalArgumentException("Invalid Spark " + (isFlex ? "Flex" : "Max")
                + " defined! Motor at id " + deviceId + " is not a Rev Motor!");
        }
    }

    public static void c_SparkMax_Destroy(long arg0) {
        SPARK_MAXES.remove((int) arg0);
    }

    public static int c_SparkMax_GetFirmwareVersion(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetDeviceId(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetMotorType(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetPeriodicFramePeriod(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetPeriodicFrameTimeout(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetControlFramePeriod(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetControlFramePeriod(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetEncoderPosition(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_RestoreFactoryDefaults(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetFollow(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_SafeFloat(float arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_EnableExternalControl(boolean arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetEnable(boolean arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetpointCommand(long arg0, float arg1, int arg2, int arg3,
        float arg4, int arg5) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetInverted(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetInverted(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartCurrentLimit(long arg0, int arg1, int arg2, int arg3) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSmartCurrentStallLimit(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSmartCurrentFreeLimit(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSmartCurrentLimitRPM(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSecondaryCurrentLimit(long arg0, float arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSecondaryCurrentLimit(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSecondaryCurrentLimitCycles(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetIdleMode(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetIdleMode(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_EnableVoltageCompensation(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetVoltageCompensationNominalVoltage(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_DisableVoltageCompensation(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetOpenLoopRampRate(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetOpenLoopRampRate(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetClosedLoopRampRate(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetClosedLoopRampRate(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_IsFollower(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetFaults(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetStickyFaults(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetFault(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetStickyFault(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetBusVoltage(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAppliedOutput(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetSimAppliedOutput(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetOutputCurrent(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetMotorTemperature(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_ClearFaults(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_BurnFlash(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetCANTimeout(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetCANMaxRetries(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_EnableSoftLimit(long arg0, int arg1, boolean arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_IsSoftLimitEnabled(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSoftLimit(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSoftLimit(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSensorType(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetLimitPolarity(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetLimitPolarity(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetLimitSwitch(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_EnableLimitSwitch(long arg0, int arg1, boolean arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_IsLimitEnabled(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAnalogPosition(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAnalogVelocity(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAnalogVoltage(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetSimAnalogPosition(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetSimAnalogVelocity(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetSimAnalogVoltage(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogPositionConversionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogVelocityConversionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAnalogPositionConversionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAnalogVelocityConversionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogInverted(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetAnalogInverted(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogAverageDepth(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAnalogAverageDepth(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogMeasurementPeriod(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAnalogMeasurementPeriod(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAnalogMode(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAnalogMode(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetEncoderPosition(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetEncoderVelocity(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetPositionConversionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetVelocityConversionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetPositionConversionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetVelocityConversionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAverageDepth(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAverageDepth(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetMeasurementPeriod(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetMeasurementPeriod(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetCountsPerRevolution(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetCountsPerRevolution(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetEncoderInverted(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetEncoderInverted(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderPosition(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAltEncoderPosition(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAltEncoderVelocity(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static void c_SparkMax_SetSimAltEncoderVelocity(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderPositionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderVelocityFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAltEncoderPositionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetAltEncoderVelocityFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderAverageDepth(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAltEncoderAverageDepth(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderMeasurementPeriod(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAltEncoderMeasurementPeriod(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderCountsPerRevolution(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAltEncoderCountsPerRevolution(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetAltEncoderInverted(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetAltEncoderInverted(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_AttemptToSetDataPortConfig(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetDataPortConfig(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDutyCyclePosition(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDutyCycleVelocity(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCyclePositionFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDutyCyclePositionFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleVelocityFactor(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDutyCycleVelocityFactor(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleInverted(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetDutyCycleInverted(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleMode(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetDutyCycleMode(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleAverageDepth(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetDutyCycleAverageDepth(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleSampleDelta(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetDutyCycleSampleDelta(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDutyCycleOffset(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDutyCycleOffset(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetHallSensorSampleRate(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetHallSensorSampleRate(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetHallSensorAverageDepth(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetHallSensorAverageDepth(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetP(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetI(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetD(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetDFilter(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetFF(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetIZone(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetOutputRange(long arg0, int arg1, float arg2, float arg3) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetP(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetI(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetD(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetDFilter(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetFF(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetIZone(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetOutputMin(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetOutputMax(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartMotionMaxVelocity(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartMotionMaxAccel(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartMotionMinOutputVelocity(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartMotionAccelStrategy(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSmartMotionAllowedClosedLoopError(long arg0, int arg1,
        float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSmartMotionMaxVelocity(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSmartMotionMaxAccel(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSmartMotionMinOutputVelocity(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSmartMotionAccelStrategy(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetSmartMotionAllowedClosedLoopError(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetIMaxAccum(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetIMaxAccum(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetIAccum(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetIAccum(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetFeedbackDevice(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetFeedbackDeviceRange(long arg0, float arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetFeedbackDeviceID(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetAPIMajorRevision() {
        return 1;
    }

    public static int c_SparkMax_GetAPIMinorRevision() {
        return 0;
    }

    public static int c_SparkMax_GetAPIBuildRevision() {
        return 0;
    }

    public static int c_SparkMax_GetAPIVersion() {
        return 1;
    }

    public static int c_SparkMax_GetLastError(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSimFreeSpeed(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetSimStallTorque(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetPositionPIDWrapEnable(long arg0, boolean arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetPositionPIDMinInput(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetPositionPIDMaxInput(long arg0, float arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetPositionPIDMaxInput(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetPositionPIDMinInput(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetPositionPIDWrapEnable(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetSparkModel(long arg0) {
        SimulatedSparkMax max = SPARK_MAXES.getKey((int) arg0);
        if (max == null) {
            throw new IllegalArgumentException("Invalid SparkMax");
        }
        if (max.isFlex()) {
            return 1;
        } else {
            return 0;
        }
    }

    public static int c_SparkMax_SetParameterFloat32(long arg0, int arg1, float arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetParameterInt32(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetParameterUint32(long arg0, int arg1, int arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_SetParameterBool(long arg0, int arg1, boolean arg2) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static float c_SparkMax_GetParameterFloat32(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetParameterInt32(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetParameterUint32(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static boolean c_SparkMax_GetParameterBool(long arg0, int arg1) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

    public static int c_SparkMax_GetMotorInterface(long arg0) {
        throw new UnsupportedOperationException("Unimplemented!");
    }

}
