package frc.robot.autonomous;

import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class DestoryerHistoryMakingTestAuton extends AutonomousRoutine {
    public DestoryerHistoryMakingTestAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        ShooterSubsystem shooter = robot.getShooter();
        SpindexerSubsystem spindexer = robot.getSpindexer();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;
        //PathPlannerPath thirdPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Test Path 1");
            secondPath = PathPlannerPath.fromPathFile("Test Path 2");
        } catch (Exception e) {
            System.err.println("Unable to load PathPlanner file - " + e.getLocalizedMessage());
            pathLoaded = false;
        }

        if (pathLoaded) {
            if (isRedAlliance) {
                firstPath = firstPath.flipPath();
                secondPath = secondPath.flipPath();
            }

            this.initialPose = firstPath.getStartingHolonomicPose().get();

            addCommands(
                    new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    new ParallelCommandGroup(
                            drivetrain.getAutoPath(firstPath),
                            // new InstantCommand(intake::deployIntake),
                            new AutoAimAndShoot(robot),
                            new SequentialCommandGroup(
                                    new WaitCommand(5.0),
                                    new InstantCommand(spindexer::runSpindexer),
                                    new WaitCommand(3.0),
                                    new InstantCommand(spindexer::stopSpindexer),
                                    new InstantCommand(shooter::stopAutoAimAndShoot))),
                    new ParallelCommandGroup(
                            drivetrain.getAutoPath(secondPath),
                            new InstantCommand(intake::runIntake)),
                    new WaitCommand(3),
                    new InstantCommand(intake::stopIntake),
                    new ParallelCommandGroup(
                            new AutoAimAndShoot(robot),
                            new SequentialCommandGroup(
                                    new WaitCommand(0.2),
                                    new InstantCommand(spindexer::runSpindexer),
                                    new WaitCommand(3.0),
                                    new InstantCommand(spindexer::stopSpindexer),
                                    new InstantCommand(shooter::stopAutoAimAndShoot))));
        }

    }
}