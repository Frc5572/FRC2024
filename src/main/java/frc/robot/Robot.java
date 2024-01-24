// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Runs tasks on Roborio in this file.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autoChooser;

    /**
     * Robnot Run type
     */
    public static enum RobotRunType {
        /** Real Robot. */
        kReal,
        /** Simulation runtime. */
        kSimulation,
        /** Replay runtime. */
        kReplay;
    }

    public RobotRunType robotRunType = RobotRunType.kReal;

    // private Ultrasonic ultrasonic = new Ultrasonic();
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }



        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
            setUseTiming(true);
            robotRunType = RobotRunType.kReal;
        } else {
            String logPath = findReplayLog();
            if (logPath == null) {
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                setUseTiming(true);
                robotRunType = RobotRunType.kSimulation;
            } else {
                // (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger
                    .addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                // Save outputs to a new log
                setUseTiming(false); // Run as fast as possible
                robotRunType = RobotRunType.kReplay;

            }
        }
        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
        // "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer(robotRunType);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands,
        // running already-scheduled commands, removing finished or interrupted commands, and
        // running
        // subsystem periodic() methods. This must be called from the robot's periodic block in
        // order for
        // anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.getAutonomousCommand().schedule();
        autoChooser = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autoChooser != null) {
            autoChooser.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autoChooser != null) {
            autoChooser.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // vision.update();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}


    private static final String environmentVariable = "AKIT_LOG_PATH";
    private static final String advantageScopeFileName = "akit-log-path.txt";

    /**
     * Finds the path to a log file for replay, using the following priorities: 1. The value of the
     * "AKIT_LOG_PATH" environment variable, if set 2. The file currently open in AdvantageScope, if
     * available 3. The result of the prompt displayed to the user
     */
    public static String findReplayLog() {
        // Read environment variables
        String envPath = System.getenv(environmentVariable);
        if (envPath != null) {
            System.out.println("Using log from " + environmentVariable
                + " environment variable - \"" + envPath + "\"");
            return envPath;
        }

        // Read file from AdvantageScope
        Path advantageScopeTempPath =
            Paths.get(System.getProperty("java.io.tmpdir"), advantageScopeFileName);
        String advantageScopeLogPath = null;
        try (Scanner fileScanner = new Scanner(advantageScopeTempPath)) {
            advantageScopeLogPath = fileScanner.nextLine();
        } catch (IOException e) {
            System.out.println("Something went wrong");
        }
        if (advantageScopeLogPath != null) {
            System.out.println("Using log from AdvantageScope - \"" + advantageScopeLogPath + "\"");
            return advantageScopeLogPath;
        }
        return null;
    }
}
