// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private FMJRobotContainer m_robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "RhsRobot2026");
        switch (Constants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        Logger.start(); // must be last
    }

    @Override
    public void robotInit() {
        m_robotContainer = new FMJRobotContainer();

        // Initialize PathPlanner
        FollowPathCommand.warmupCommand();

        // // if (DriverStation.isFMSAttached()) {
        // //Start WPILib Data Log
        // DataLogManager.start(); // stores logs in either USB(diretory logs) or
        // roborio drive(/home/lvuser/logs)
        // DriverStation.startDataLog(DataLogManager.getLog());

        // // Start CTRE Data Log - logging will automatically start for FRC match
        // SignalLogger.setPath("/media/sda1/ctre-logs"); // we need to valid this
        // location
        // SignalLogger.start();
        // // }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.robotPeriodic();
    }

    @Override
    public void disabledInit() {
        Constants.hasInitializedFromVision = false;
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.autonomousInit();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        LimelightHelpers.setPipelineIndex("limelight-a", 0);
        LimelightHelpers.setPipelineIndex("limelight-b", 0);
        LimelightHelpers.SetThrottle("limelight-a", 0);
        LimelightHelpers.SetThrottle("limelight-b", 0);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_robotContainer.autonomousExit();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        LimelightHelpers.setPipelineIndex("limelight-a", 0);
        LimelightHelpers.setPipelineIndex("limelight-b", 0);
        LimelightHelpers.SetThrottle("limelight-a", 0);
        LimelightHelpers.SetThrottle("limelight-b", 0);

        m_robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.teleopPeriodic();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
