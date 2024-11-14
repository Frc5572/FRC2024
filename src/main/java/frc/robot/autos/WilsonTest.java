package frc.robot.autos;

import org.littletonrobotics.junction.Logger;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class WilsonTest {

    private final AutoFactory factory;
    private final Swerve swerve;
    private final Shooter shooter;

    public WilsonTest(Swerve swerve, ElevatorWrist ew, Shooter shooter, Intake intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        AutoBindings bindings = new AutoBindings();
        bindings.bind("shoot", intake.runIndexerMotor(1.0));
        this.factory = Choreo.createAutoFactory(swerve, swerve::getPose, swerve::choreoController,
            () -> true, bindings);
    }

    public Command cmd() {
        var loop = factory.newLoop("myAuto");
        AutoTrajectory trajectory = factory.trajectory("short", loop);
        Pose2d initialPose = trajectory.getInitialPose().get();
        Command resetPose = swerve.runOnce(() -> swerve.resetOdometry(initialPose));
        loop.enabled().onTrue(trajectory.cmd());
        return Commands.sequence(Commands.runOnce(() -> Logger.recordOutput("AutoState", 1)),
            resetPose, Commands.runOnce(() -> Logger.recordOutput("AutoState", 2)), loop.cmd(),
            Commands.runOnce(() -> Logger.recordOutput("AutoState", 3)));
    }

    public Command cmd2() {
        var loop = factory.newLoop("myAuto2");
        AutoTrajectory trajectory = factory.trajectory("simple", loop);
        Pose2d initialPose = trajectory.getInitialPose().get();
        Command resetPose = swerve.runOnce(() -> swerve.resetOdometry(initialPose));
        loop.enabled().onTrue(trajectory.cmd());
        return Commands.sequence(Commands.runOnce(() -> Logger.recordOutput("AutoState", 1)),
            resetPose, Commands.runOnce(() -> Logger.recordOutput("AutoState", 2)), loop.cmd(),
            Commands.runOnce(() -> Logger.recordOutput("AutoState", 3)));
    }

    public Command cmd3() {
        var loop = factory.newLoop("myAuto3");
        AutoTrajectory trajectory = factory.trajectory("testtree", loop);
        Pose2d initialPose = trajectory.getInitialPose().get();
        Command resetPose = swerve.runOnce(() -> swerve.resetOdometry(initialPose));
        loop.enabled().onTrue(trajectory.cmd());
        return Commands
            .sequence(Commands.runOnce(() -> Logger.recordOutput("AutoState", 1)), resetPose,
                Commands.runOnce(() -> Logger.recordOutput("AutoState", 2)), loop.cmd(),
                Commands.runOnce(() -> Logger.recordOutput("AutoState", 3)))
            .alongWith(shooter.spit().withTimeout(5.0));
    }

}
