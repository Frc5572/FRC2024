package frc.lib.util;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve setpoint generator based on a version created by FRC team 254.
 *
 * <p>
 * Takes a prior setpoint, a desired setpoint, and outputs a new setpoint that respects all the
 * kinematic constraints on module rotation and wheel velocity/torque, as well as preventing any
 * forces acting on a module's wheel from exceeding the force of friction.
 */
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics m_kinematics;
    private final double m_maxDriveSpeed; // m/s
    private final double m_maxSpeedDiff; // m/s/s
    private final double m_maxAngleDiff; // rad/s

    public SwerveSetpointGenerator(
        final SwerveDriveKinematics kinematics,
        final double maxDriveSpeed,
        final double maxModuleAcceleration,
        final double maxModuleSteeringRate) {
        m_kinematics = kinematics;
        m_maxDriveSpeed = maxDriveSpeed;
        m_maxSpeedDiff = maxModuleAcceleration * LoggedRobot.defaultPeriodSecs;
        m_maxAngleDiff = maxModuleSteeringRate * LoggedRobot.defaultPeriodSecs;
    }

    /**
     * Returns module states that result in ChassisSpeeds as close to |desiredChassisSpeeds| as
     * possible without exceeding |allowedScrub|.
     *
     * @param desiredChassisSpeeds Chassis speeds to try to achieve while staying in scrub and
     *        feasibility limits.
     * @param allowedScrub Maximum scrub permissible in m/s.
     */
    public SwerveSetpoint getFeasibleSetpoint(
        final SwerveSetpoint previousSetpoint,
        final ChassisSpeeds desiredChassisSpeeds,
        final double allowedScrub) {
        final var desiredSetpoint = getDesaturatedSwerveSetpoint(desiredChassisSpeeds);

        // Binary search between linearly interpolated current and desired chassis speeds that
        // doesn't
        // exceed feasibility limits or the desired amount of module scrub.
        double lowerBound = 0.01; // Non-zero to ensure we always make some progress.
        double upperBound = 1.0;

        // Optimistically use upper bound as setpoint.
        SwerveSetpoint bestSetpoint =
            optimizeWithSlewRateLimit(
                getDesaturatedSwerveSetpoint(
                    interpolateChassisSpeeds(
                        previousSetpoint.chassisSpeeds, desiredSetpoint.chassisSpeeds,
                        upperBound)).moduleStates,
                previousSetpoint.moduleStates,
                m_maxSpeedDiff,
                m_maxAngleDiff);
        final double maxUpperBoundScrub = computeMaxModuleScrubs(bestSetpoint.moduleStates);
        if (maxUpperBoundScrub <= allowedScrub) {
            return bestSetpoint;
        }

        // Begin binary search.
        for (int n = 0; n < 10; n++) {
            // Alpha of 1.0 means to fully use the desired chassis speeds..
            // 0.0 means to fully use the current chassis speeds.
            final double alpha = (lowerBound + upperBound) * 0.5;
            final SwerveSetpoint testSetpoint =
                optimizeWithSlewRateLimit(
                    getDesaturatedSwerveSetpoint(
                        interpolateChassisSpeeds(
                            previousSetpoint.chassisSpeeds, desiredSetpoint.chassisSpeeds,
                            alpha)).moduleStates,
                    previousSetpoint.moduleStates,
                    m_maxSpeedDiff,
                    m_maxAngleDiff);

            final double maxScrub = computeMaxModuleScrubs(testSetpoint.moduleStates);
            if (maxScrub > allowedScrub) {
                upperBound = alpha;
            } else {
                lowerBound = alpha;
                bestSetpoint = testSetpoint;
            }
        }
        return bestSetpoint;
    }

    /** Returns the SwerveSetpoint for the given |chassisSpeeds| after desaturating wheel speeds. */
    private SwerveSetpoint getDesaturatedSwerveSetpoint(final ChassisSpeeds chassisSpeeds) {
        final SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxDriveSpeed);
        return new SwerveSetpoint(m_kinematics.toChassisSpeeds(states), states);
    }

    private static ChassisSpeeds interpolateChassisSpeeds(
        final ChassisSpeeds a, final ChassisSpeeds b, final double alpha) {
        return new ChassisSpeeds(
            (1 - alpha) * a.vxMetersPerSecond + alpha * b.vxMetersPerSecond,
            (1 - alpha) * a.vyMetersPerSecond + alpha * b.vyMetersPerSecond,
            (1 - alpha) * a.omegaRadiansPerSecond + alpha * b.omegaRadiansPerSecond);
    }

    private SwerveSetpoint optimizeWithSlewRateLimit(
        final SwerveModuleState[] targetStates,
        final SwerveModuleState[] currentStates,
        final double maxSpeedDiff,
        final double maxAngleDiff) {
        final SwerveModuleState[] optimizedStates = new SwerveModuleState[targetStates.length];
        for (int i = 0; i < targetStates.length; i++) {
            // Optimize first to ensure steering rate differences are minimized.
            optimizedStates[i] = optimizeModule(targetStates[i], currentStates[i]);

            // Apply acceleration limit.
            final double speedDiff =
                optimizedStates[i].speedMetersPerSecond - currentStates[i].speedMetersPerSecond;
            if (speedDiff > maxSpeedDiff) {
                optimizedStates[i].speedMetersPerSecond =
                    currentStates[i].speedMetersPerSecond + maxSpeedDiff;
            }
            if (speedDiff < -maxSpeedDiff) {
                optimizedStates[i].speedMetersPerSecond =
                    currentStates[i].speedMetersPerSecond - maxSpeedDiff;
            }

            // Apply steering rate limit.
            final double angleDiff =
                optimizedStates[i].angle.getRadians() - currentStates[i].angle.getRadians();
            if (angleDiff > maxAngleDiff) {
                optimizedStates[i].angle =
                    new Rotation2d(currentStates[i].angle.getRadians() + maxAngleDiff);
            }
            if (angleDiff < -maxAngleDiff) {
                optimizedStates[i].angle =
                    new Rotation2d(currentStates[i].angle.getRadians() - maxAngleDiff);
            }

            // Re-optimize final result.
            optimizedStates[i] = optimizeModule(optimizedStates[i], currentStates[i]);
        }
        return new SwerveSetpoint(m_kinematics.toChassisSpeeds(optimizedStates), optimizedStates);
    }

    /** Returns the amount of scrub for each module relative to overall chassis movement. */
    public double computeMaxModuleScrubs(final SwerveModuleState[] states) {
        final var idealStates =
            m_kinematics.toSwerveModuleStates(m_kinematics.toChassisSpeeds(states));
        double maxScrub = 0.0;
        for (int i = 0; i < states.length; i++) {
            final var cart1 =
                pol2cart(
                    idealStates[i].speedMetersPerSecond, idealStates[i].angle.getRadians());
            final var cart2 =
                pol2cart(states[i].speedMetersPerSecond, states[i].angle.getRadians());
            final var dx = cart2.getFirst() - cart1.getFirst();
            final var dy = cart2.getSecond() - cart1.getSecond();
            final var scrub = Math.sqrt(dx * dx + dy * dy);
            maxScrub = Math.max(maxScrub, scrub);
        }
        return maxScrub;
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Places the result within [-pi, pi) of the current
     * state. Additionally, keeps the same steering angle if the speed zero.
     *
     * @param desiredState The desired state.
     * @param currentState The current state.
     */
    public static SwerveModuleState optimizeModule(
        final SwerveModuleState desiredState, final SwerveModuleState currentState) {
        double targetAngle =
            placeInScope(desiredState.angle.getRadians(),
                currentState.angle.getRadians());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentState.angle.getRadians();
        if (Math.abs(delta) > 0.5 * Math.PI) {
            targetSpeed = -targetSpeed;
            targetAngle =
                delta > 0.5 * Math.PI ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
        }
        // Don't steer unnecessarily at singularity.
        if (targetSpeed == 0.0) {
            targetAngle = currentState.angle.getRadians();
        }
        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }

    public static class SwerveSetpoint {
        public final ChassisSpeeds chassisSpeeds;
        public SwerveModuleState[] moduleStates;

        public SwerveSetpoint(final ChassisSpeeds chassisSpeeds,
            final SwerveModuleState[] moduleStates) {
            this.chassisSpeeds = chassisSpeeds;
            this.moduleStates = moduleStates;
        }

        @Override
        public String toString() {
            String ret = chassisSpeeds.toString() + "\n";
            for (int i = 0; i < moduleStates.length; ++i) {
                ret += "  " + moduleStates[i].toString() + "\n";
            }
            return ret;
        }
    }

    private static Pair<Double, Double> pol2cart(final double r, final double theta) {
        final double x = r * Math.cos(theta);
        final double y = r * Math.sin(theta);
        return new Pair<>(x, y);
    }

    /** Returns |angle| placed within within [-pi, pi) of |referenceAngle|. */
    private static double placeInScope(final double angle, final double referenceAngle) {
        return referenceAngle + constrainAngleNegPiToPi(angle - referenceAngle);
    }

    /** Constrains an angle to be within [-pi, pi). */
    private static double constrainAngleNegPiToPi(final double angle) {
        double x = (angle + Math.PI) % (2.0 * Math.PI);
        if (x < 0.0) {
            x += 2.0 * Math.PI;
        }
        return x - Math.PI;
    }
}
