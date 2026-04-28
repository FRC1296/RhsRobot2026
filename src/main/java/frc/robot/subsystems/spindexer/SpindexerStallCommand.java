// SpindexerStallCommand.java - void everything and rewrite:

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class SpindexerStallCommand extends Command {
    private SpindexerSubsystem spindexer;
    private FeederSubsystem feeder;
    private long startTime;
    private static final double TIME_TO_REVERSE_MS = 250;

    public SpindexerStallCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        addRequirements(spindexer,feeder);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        spindexer.reverseSpindexer();
        feeder.reverseFeeder();

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
        feeder.runFeeder();
    }
}