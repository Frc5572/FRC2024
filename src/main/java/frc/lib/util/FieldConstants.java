
package frc.lib.util;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;
import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {

    public static double fieldLength = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);
    public static double wingX = Units.inchesToMeters(229.201);
    public static double podiumX = Units.inchesToMeters(126.75);
    public static double startingLineX = Units.inchesToMeters(74.111);

    public static Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

    /** Staging locations for each note */
    public static final class StagingLocations {
        public static double centerlineX = fieldLength / 2.0;

        // need to update
        public static double centerlineFirstY = Units.inchesToMeters(29.638);
        public static double centerlineSeparationY = Units.inchesToMeters(66);
        public static double spikeX = Units.inchesToMeters(114);
        // need
        public static double spikeFirstY = Units.inchesToMeters(161.638);
        public static double spikeSeparationY = Units.inchesToMeters(57);

        public static Translation2d[] centerlineTranslations = new Translation2d[5];
        public static Translation2d[] spikeTranslations = new Translation2d[3];

        static {
            for (int i = 0; i < centerlineTranslations.length; i++) {
                centerlineTranslations[i] =
                    new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
            }
        }

        static {
            for (int i = 0; i < spikeTranslations.length; i++) {
                spikeTranslations[i] =
                    new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
            }
        }
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

        /** Center of the speaker opening (blue alliance) */
        public static Pose2d centerSpeakerOpening =
            new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());
    }

    // corners (blue alliance origin)
    public static Translation3d topRightSpeaker = new Translation3d(Units.inchesToMeters(18.055),
        Units.inchesToMeters(238.815), Units.inchesToMeters(83.091));

    public static Translation3d topLeftSpeaker = new Translation3d(Units.inchesToMeters(18.055),
        Units.inchesToMeters(197.765), Units.inchesToMeters(83.091));

    public static Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
    public static double centerSpeaker = topLeftSpeaker.getZ() - bottomLeftSpeaker.getZ();

    public static double aprilTagWidth = Units.inchesToMeters(6.50);
    public static AprilTagFieldLayout aprilTags;

    static {
        try {
            aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with the origin at
     * the rightmost point on the BLUE ALLIANCE wall.
     *
     * @param pose Initial Pose
     * @return Pose2d flipped to Red Alliance
     */
    public static Pose2d allianceFlip(Pose2d pose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                pose = new Pose2d(fieldLength - pose.getX(), pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
            }
        }
        return pose;
    }

    private static double sign(Translation2d p1, Translation2d p2, Translation2d p3) {
        return (p1.getX() - p3.getY()) * (p2.getY() - p3.getY())
            - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }

    private static boolean PointInTriangle(Translation2d pt, Translation2d v1, Translation2d v2,
        Translation2d v3) {
        double d1, d2, d3;
        boolean has_neg, has_pos;

        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    public static boolean underStage(Pose2d pose) {
        Translation2d p = pose.getTranslation();
        return PointInTriangle(p, new Translation2d(153, 121), new Translation2d(98, 218),
            new Translation2d(231, 218));

    }
}
