package frc.robot.autonomous;

import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.commands.AutoAimAndShootMoving;
import frc.robot.commands.SmartMove;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class LeftToDepot extends AutonomousRoutine {
    public LeftToDepot(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        SpindexerSubsystem spindexer = robot.getSpindexer();
        FeederSubsystem feeder = robot.getFeeder();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;
        PathPlannerPath thirdPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Left To Shoot Pos");
            secondPath = PathPlannerPath.fromPathFile("Collect Depot Ball");
            thirdPath = PathPlannerPath.fromPathFile("Depot Ball To Shoot Pos");
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
                    new InstantCommand(() -> SmartMove.move(drivetrain, initialPose.getX(), initialPose.getY(), 0.0)),
                   // Commands.runOnce(intake::deployIntake,intake),
                    drivetrain.getAutoPath(firstPath),
                    Commands.runOnce(feeder::runFeeder, feeder),
                    Commands.runOnce(spindexer::runSpindexer, spindexer),
                    new WaitCommand(4.0),
                    Commands.runOnce(feeder::stopFeeder, feeder),
                    Commands.runOnce(spindexer::stopSpindexer, spindexer),
                    new ParallelCommandGroup(drivetrain.getAutoPath(secondPath),
                            Commands.runOnce(intake::runIntake, intake)),
                    new WaitCommand(2.0),
                     Commands.runOnce(intake::stopIntake, intake),
                    drivetrain.getAutoPath(thirdPath),
                     Commands.runOnce(feeder::runFeeder, feeder),
                    Commands.runOnce(spindexer::runSpindexer, spindexer),
                    new WaitCommand(5.0),
                    Commands.runOnce(feeder::stopFeeder, feeder),
                    Commands.runOnce(spindexer::stopSpindexer, spindexer)

                            );
                    
        }

    }
}