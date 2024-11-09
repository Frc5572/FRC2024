package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class WilsonTest {

    public static Command cmd(Swerve swerve) {
        AutoFactory factory = swerve.getFactory();
        var loop = factory.newLoop("myAuto");
        AutoTrajectory trajectory = factory.trajectory("WilsonTest", loop);
        Pose2d initialPose = trajectory.getInitialPose().get();
        Command resetPose = swerve.run(() -> swerve.resetOdometry(initialPose));
        loop.enabled().onTrue(trajectory.cmd());
        return resetPose.andThen(loop.cmd());
    }

}
