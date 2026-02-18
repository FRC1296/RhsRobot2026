package frc.robot.autonomous;

import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class DestoryerHistoryMakingTestAuton extends AutonomousRoutine {
    public DestoryerHistoryMakingTestAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        ShooterSubsystem shooter = robot.getShooter();
        FeederSubsystem feeder = robot.getFeeder();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;
        PathPlannerPath thirdPath = null;
        boolean pathLoaded = true;

        try {
            firstPath = PathPlannerPath.fromPathFile("Test Path 1");
            secondPath = PathPlannerPath.fromPathFile("Test Path 2");
            thirdPath = PathPlannerPath.fromPathFile("");

        } catch (Exception e) {
            System.err.println("Unable to load PathPlanner file - " + e.getLocalizedMessage());
            pathLoaded = false;
        }
        if (pathLoaded) {
            if (isRedAlliance) {
                firstPath = firstPath.flipPath();
                secondPath = secondPath.flipPath();
                thirdPath = thirdPath.flipPath();

            }

            this.initialPose = firstPath.getStartingHolonomicPose().get();

            addCommands(
                    new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    new ParallelCommandGroup(
                            drivetrain.getAutoPath(firstPath),
                            // new InstantCommand(intake::deployIntake),
                            new AutoAimAndShoot(robot)),
                    new WaitCommand(0.5),
                    new InstantCommand(feeder::runSpindexer),
                    new WaitCommand(3),
                    new ParallelCommandGroup(
                            new InstantCommand(feeder::stopSpindexer),
                            new InstantCommand(shooter::stopShooterAutoInterpolate)),
                    new ParallelCommandGroup(
                            drivetrain.getAutoPath(secondPath),
                            new InstantCommand(intake::runIntake)),
                    new WaitCommand(3),
                    new InstantCommand(intake::stopIntake),
                    new AutoAimAndShoot(robot),
                    new InstantCommand(feeder::runSpindexer),
                    new WaitCommand(3),
                    new ParallelCommandGroup(
                            new InstantCommand(feeder::stopSpindexer),
                            new InstantCommand(shooter::stopShooterAutoInterpolate)));

        }

    }
}