// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final FMJRobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new FMJRobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {
    LimelightHelpers.setPipelineIndex("limelight-a", 1);
    LimelightHelpers.setPipelineIndex("limelight-b", 1);
    LimelightHelpers.SetThrottle("limelight-a", 200);
    LimelightHelpers.SetThrottle("limelight-b", 200);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.setInitialPose(1.3, 2.4);

    LimelightHelpers.setPipelineIndex("limelight-a", 0);
    LimelightHelpers.setPipelineIndex("limelight-b", 0);
    LimelightHelpers.SetThrottle("limelight-a", 0);
    LimelightHelpers.SetThrottle("limelight-b", 0);
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
