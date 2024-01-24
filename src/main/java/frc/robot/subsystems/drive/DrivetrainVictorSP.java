package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * Drivetrain VictorSP
 */
public class DrivetrainVictorSP implements DrivetrainIO {
    private final VictorSP left1 = new VictorSP(4);
    private final VictorSP left2 = new VictorSP(5);

    private final VictorSP right1 = new VictorSP(6);
    private final VictorSP right2 = new VictorSP(7);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * Drivetrain VictorSP
     */
    public DrivetrainVictorSP() {
        left1.addFollower(left2);
        right1.addFollower(right2);
        right1.setInverted(true);
        right2.setInverted(true);
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.gyroYaw = Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Drive Voltage
     */
    public void setDriveVoltage(double lvolts, double rvolts) {
        left1.set(lvolts);
        right1.set(rvolts);
    }

}
