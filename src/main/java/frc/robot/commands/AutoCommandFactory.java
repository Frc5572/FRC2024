package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class AutoCommandFactory {

    public static AutoLoop p8765(AutoFactory factory, Swerve swerve, Intake intake, Shooter shooter,
        ElevatorWrist elevatorWrist) {
        final AutoLoop loop = factory.newLoop("P123");

        Trigger noteInIndexer = new Trigger(() -> intake.getIndexerBeamBrakeStatus()).debounce(0.25,
            Debouncer.DebounceType.kRising);
        Trigger noteInIntake = new Trigger(() -> intake.getintakeBeamBrakeStatus()).debounce(0.25,
            Debouncer.DebounceType.kRising);
        Trigger abort = noteInIndexer.negate().and(noteInIntake.negate());
        Trigger dumpOrNot =
            new Trigger(() -> RobotContainer.dumpNotes.getEntry().getBoolean(false));

        final AutoTrajectory startToS0 = factory.trajectory("P8765-initial", loop);
        final AutoTrajectory s0ToP8 = factory.trajectory("P8765-p8", loop);
        final AutoTrajectory path2 = factory.trajectory("P8765-p7", loop);
        final AutoTrajectory path3 = factory.trajectory("P8765-p6", loop);
        final AutoTrajectory s0P8Dump = factory.trajectory("P8765-p8-dump", loop);
        final AutoTrajectory p8p7Dump = factory.trajectory("P8765-p7-dump", loop);
        final AutoTrajectory p7p6Dump = factory.trajectory("P8765-p6-dump", loop);
        final AutoTrajectory p6p5Dump = factory.trajectory("P8765-p5-dump", loop);

        loop.enabled().whileTrue(shooter.shootSpeaker().withName("Run Shooter always"));
        loop.enabled().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(startToS0.getInitialPose().orElseGet(() -> {
                loop.kill();
                return new Pose2d();
            }))).andThen(startToS0.cmd()).withName("P8765 entry point"));
        startToS0.active().onTrue(elevatorWrist.goToPosition(
            Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, Rotation2d.fromDegrees(31.0)));
        startToS0.done().and(() -> elevatorWrist.atGoal())
            .onTrue(Commands.waitSeconds(.1).andThen(CommandFactory.Auto.runIndexer(intake)));
        startToS0.done().and(abort).and(dumpOrNot).onTrue(s0P8Dump.cmd());
        startToS0.done().and(abort).and(dumpOrNot.negate()).onTrue(s0ToP8.cmd());

        dumpOrNot.and(s0P8Dump.active()).onTrue(elevatorWrist.homePosition())
            .whileTrue(CommandFactory.intakeNote(intake));
        dumpOrNot.and(s0P8Dump.done()).and(abort).onTrue(p8p7Dump.cmd());
        dumpOrNot.and(s0P8Dump.done()).and(abort.negate())
            .onTrue(CommandFactory.Auto.runIndexer(intake));

        dumpOrNot.and(p8p7Dump.active()).whileTrue(CommandFactory.intakeNote(intake));
        dumpOrNot.and(p8p7Dump.done()).and(abort).onTrue(p7p6Dump.cmd());
        dumpOrNot.and(p8p7Dump.done()).and(abort.negate())
            .onTrue(CommandFactory.Auto.runIndexer(intake));

        dumpOrNot.and(p7p6Dump.active()).whileTrue(CommandFactory.intakeNote(intake));
        dumpOrNot.and(p7p6Dump.done()).and(abort).onTrue(p6p5Dump.cmd());
        dumpOrNot.and(p7p6Dump.done()).and(abort.negate())
            .onTrue(CommandFactory.Auto.runIndexer(intake));

        dumpOrNot.and(p6p5Dump.active()).whileTrue(CommandFactory.intakeNote(intake));
        dumpOrNot.and(p6p5Dump.done()).and(abort).onTrue(p8p7Dump.cmd());
        dumpOrNot.and(p6p5Dump.done()).and(abort.negate())
            .onTrue(CommandFactory.Auto.runIndexer(intake));

        return loop;
    }

    public static AutoLoop none(AutoFactory factory) {
        final AutoLoop loop = factory.newLoop("Empty Auto");

        return loop;
    }


    public static AutoLoop fivePieceAutoTriggerSeg(AutoFactory factory, Swerve swerve,
        Intake intake, Shooter shooter, ElevatorWrist elevatorWrist) {
        final AutoLoop loop = factory.newLoop("fivePieceAuto");


        Trigger noteInIndexer = new Trigger(() -> intake.getIndexerBeamBrakeStatus()).debounce(0.25,
            Debouncer.DebounceType.kRising);
        Trigger noteInIntake = new Trigger(() -> intake.getintakeBeamBrakeStatus()).debounce(0.25,
            Debouncer.DebounceType.kRising);
        Trigger abort = noteInIndexer.negate().and(noteInIntake.negate());

        // This uses segments that all have predefined handoff points.
        // These handoff points follow a naming convention
        // C1, C2, C3: The 3 close notes, C1 having the greatest y value
        // M1, M2, M3, M4, M5: The 5 middle notes, M1 having the greatest y value
        // S1, S2, S3: 3 arbitrary shooting positions that are near the stage, S1 having the
        // greatest y
        // value
        // AMP, SUB, SRC: The 3 starting positions

        // Try to load all the trajectories we need
        final AutoTrajectory ampToC1 = factory.trajectory("ampToC1", loop);
        final AutoTrajectory c1ToM1 = factory.trajectory("c1ToM1", loop);
        final AutoTrajectory m1ToS1 = factory.trajectory("m1ToS1", loop);
        final AutoTrajectory m1ToM2 = factory.trajectory("m1ToM2", loop);
        final AutoTrajectory m2ToS1 = factory.trajectory("m2ToS2", loop);
        final AutoTrajectory s1ToC2 = factory.trajectory("s1ToC2", loop);
        final AutoTrajectory c2ToC3 = factory.trajectory("c2ToC3", loop);


        // // entry point for the auto
        // // resets the odometry to the starting position,
        // // then shoots the starting note,
        // // then runs the trajectory to the first close note while extending the intake
        // loop.enabled().onTrue(swerve.resetOdometry(ampToC1.getInitialPose().orElseGet(() -> {
        // loop.kill();
        // return new Pose2d();
        // })).andThen(shooter.shootSpeaker(),
        // Commands.race(CommandFactory.runIntake(intake, indexer, null), ampToC1.cmd(),
        // aimFor(ampToC1.getFinalPose().orElseGet(Pose2d::new))))
        // .withName("fivePieceAuto entry point"));

        // // // spinnup the shooter while no other command is running
        // loop.enabled().and(shooter.isShooting.negate()).onTrue(shooter.spit());

        // // // shoots the note if the robot has it, then runs the trajectory to the first middle
        // note
        // ampToC1.done().and(indexer.noteInIndexer).onTrue(CommandFactory
        // .shootSpeaker(shooter, indexer).until(indexer.noteNotInIndexer).andThen(c1ToM1.cmd()));

        // // // extends the intake while traveling towards the first middle note
        // // // if the robot has the note, it goes back to shoot it,
        // // // otherwise it goes to the next middle note
        // c1ToM1.atTime("intake").onTrue(CommandFactory.runIntake(intake, null));
        // c1ToM1.done().and(abort).onTrue(m1ToS1.cmd());
        // c1ToM1.done().and(abort.negate()).onTrue(m1ToM2.cmd());

        // // // aims the shooter while traveling to shoot
        // m1ToS1.active()
        // .whileTrue(elevatorWrist.followPosition(
        // () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, () -> elevatorWrist
        // .getAngleFromDistance(m1ToS1.getFinalPose().orElseGet(Pose2d::new))));
        // m1ToS1.done().and(noteInIndexer).onTrue(
        // CommandFactory.shootSpeaker(shooter).until(noteNotInIndexer).andThen(s1ToC2.cmd()));
        // m1ToS1.done().onTrue(m1ToM2.cmd().until(abort));

        // // extends the intake while traveling towards the second middle note
        // // go back to shoot no matter what
        // m1ToM2.active().whileTrue(intake());
        // m1ToM2.done().onTrue(m2ToS1.cmd());

        // // aims the shooter while traveling to shoot
        // m2ToS1.active().whileTrue(aimFor(m2ToS1.getFinalPose().orElseGet(Pose2d::new)));
        // m2ToS1.done().onTrue(shootIfGp());
        // m2ToS1.done().onTrue(s1ToC2.cmd().after(noGp(loop)));

        // // extends the intake while traveling towards the second close note
        // // if the robot has the note, it shoots it
        // // otherwise it goes to the third close note
        // s1ToC2.active().whileTrue(intake());
        // s1ToC2.active().whileTrue(aimFor(s1ToC2.getFinalPose().orElseGet(Pose2d::new)));
        // s1ToC2.done().onTrue(shootIfGp());
        // s1ToC2.done().onTrue(c2ToC3.cmd().after(noGp(loop)));

        // // extends the intake while traveling towards the third close note
        // // if the robot has the note, it shoots it
        // c2ToC3.active().whileTrue(intake());
        // c2ToC3.done().onTrue(shootIfGp());

        return loop;
    }
}
