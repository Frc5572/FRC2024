package frc.robot.subsystems.elevatorWrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ElevatorWristReal implements ElevatorWristIO {
    public final TalonFX elevatorMotor = new TalonFX(Constants.Motors.ElevatorWrist.TALON_ID);

    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.NEO_ID, MotorType.kBrushless);

    public ElevatorWristReal() {

    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {}

    @Override
    public void setElevatorVoltage(double voltage) {}

    @Override
    public void setWristVoltage(double voltage) {}

}
