package frc.robot.subsystems.swerve.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.mod.ModuleConfig;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleAngleIO;
import frc.robot.subsystems.swerve.mod.SwerveModuleDriveIO;

/** Swerve Subsystem */
public final class Swerve extends SubsystemBase {

    public static final Lock odometryLock = new ReentrantLock();

    private final SwerveIO io;
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    private final SwerveModule[] modules;

    /** Swerve Subsystem */
    public Swerve(SwerveIO io, ModuleConfig[] modules,
        BiFunction<Integer, ModuleConfig, Pair<SwerveModuleAngleIO, SwerveModuleDriveIO>> modFunc) {
        super("Swerve");
        this.io = io;
        this.modules = new SwerveModule[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var modIO = modFunc.apply(i, modules[i]);
            this.modules[i] = new SwerveModule(modules[i], modIO.getFirst(), modIO.getSecond());
        }
    }

    public Swerve(SwerveIO io, ModuleConfig[] modules,
        BiFunction<Integer, ModuleConfig, SwerveModuleAngleIO> angleFunc,
        BiFunction<Integer, ModuleConfig, SwerveModuleDriveIO> driveFunc) {
        this(io, modules, (i, conf) -> Pair.of(angleFunc.apply(i, conf), driveFunc.apply(i, conf)));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        for (var module : modules) {
            module.periodic();
        }


    }

}
