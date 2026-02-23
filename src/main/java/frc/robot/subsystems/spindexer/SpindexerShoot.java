package frc.robot.subsystems.spindexer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FMJRobotContainer;

public class SpindexerShoot extends Command {
    private SpindexerSubsystem spindexer;
    private DoubleSubscriber speedSubscriber;

    public SpindexerShoot(FMJRobotContainer robot) {
        this.spindexer = robot.getSpindexer();
        addRequirements(spindexer);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable("Robot Data");
        NetworkTable driveTable = robotTable.getSubTable("Drive Subsystem");
        speedSubscriber = driveTable.getDoubleTopic("Robot Speed").subscribe(0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (speedSubscriber.getAsDouble() <= 0.1) {
            spindexer.runSpindexer();
        } else {
            spindexer.stopSpindexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
