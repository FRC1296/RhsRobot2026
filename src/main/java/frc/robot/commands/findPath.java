package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FindPath {

    public static Command pathFindToPose(double targetX, double targetY, double targetRotationDegrees) {

        Pose2d targetPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetRotationDegrees));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                2.5, 2.0,
                Math.toRadians(360), Math.toRadians(360));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );

        return pathfindingCommand.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }
}