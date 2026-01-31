package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class SmartMove {

    private static final double PATH_THRESHOLD = 0.35; // meters

    public static Command move(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, double targetRotationDegrees) {
        Pose2d targetPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetRotationDegrees));
        Pose2d current = drivetrain.getPose();
        double distance = current.getTranslation().getDistance(targetPose.getTranslation());

        if (distance > PATH_THRESHOLD) {
            // Path-on-the-fly → then PID
            return FindPath.pathFindToPose(targetX, targetY, targetRotationDegrees)
                    .andThen(new PIDAlign(drivetrain, targetPose)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
        } else {
            // Already close → PID only
            return new PIDAlign(drivetrain, targetPose).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
        }
    }
}
