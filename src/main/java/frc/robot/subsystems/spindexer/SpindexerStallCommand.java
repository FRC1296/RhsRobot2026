// SpindexerStallCommand.java - void everything and rewrite:

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;

public class SpindexerStallCommand extends Command {
    private SpindexerSubsystem spindexer;
    private long startTime;
    private static final double TIME_TO_REVERSE_MS = 250;

    public SpindexerStallCommand(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        spindexer.reverseSpindexer();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= TIME_TO_REVERSE_MS;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.runSpindexer();
    }
}