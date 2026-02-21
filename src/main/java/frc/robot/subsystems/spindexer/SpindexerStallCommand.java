// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpindexerStallCommand extends Command {
  private SpindexerSubsystem spindexerSubsystem;
  private BooleanSubscriber stallDetector;
  /** Creates a new SpindexerStallCommand. */
  public SpindexerStallCommand(SpindexerSubsystem spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    spindexerSubsystem = spindexer;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable("Robot Data");
        NetworkTable feederTable = robotTable.getSubTable("Feeder Subsystem");
    stallDetector = feederTable.getBooleanTopic("Spindexer Stall").subscribe(false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexerSubsystem.stopSpindexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexerSubsystem.reverseSpindexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //spindexerSubsystem.runSpindexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = stallDetector.get();
    return isDone;
  }
}
