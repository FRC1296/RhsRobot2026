package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.commands.SmartMove;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.vision.LocalizationHelpers;

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
            firstPath = PathPlannerPath.fromPathFile("Left To Midway");
            secondPath = PathPlannerPath.fromPathFile("Midway To Depot");
            thirdPath = PathPlannerPath.fromPathFile("Depot To Shoot Pos");

            
         
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
                    new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")),
                    new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    new InstantCommand(() -> SmartMove.move(drivetrain, initialPose.getX(), initialPose.getY(), 0.0)),
                    Commands.runOnce(intake::deployIntake,intake),
                    new ParallelCommandGroup(
                        drivetrain.getAutoPath(firstPath),
                        Commands.runOnce(intake::runIntake, intake)
                    ),
                    drivetrain.getAutoPath(secondPath),
                    new WaitCommand(1),
                    Commands.runOnce(intake::stopIntake, intake),
                    Commands.runOnce(intake::undeployIntake,intake),
                    //drivetrain.getAutoPath(firstPath),
                    // new ParallelCommandGroup(drivetrain.getAutoPath(secondPath),
                    //         Commands.runOnce(intake::runIntake, intake)),
                    drivetrain.getAutoPath(thirdPath),
                    new InstantCommand(feeder::runFeeder, feeder),
                    new InstantCommand(spindexer::runSpindexer, spindexer),
                    new InstantCommand(intake::runIntakeReverse, intake)
                    // new WaitCommand(2)
                    // Commands.runOnce(feeder::runFeeder, feeder),
                    // Commands.runOnce(spindexer::runSpindexer, spindexer),
                    // new WaitCommand(9.0),
                    // Commands.runOnce(feeder::stopFeeder, feeder),
                    // Commands.runOnce(spindexer::stopSpindexer, spindexer)
            );
                    
        }

    }

}