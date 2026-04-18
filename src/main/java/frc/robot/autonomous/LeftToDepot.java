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
import frc.robot.subsystems.intake.AgitateBalls;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
//import frc.robot.subsystems.vision.LocalizationHelpers;

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
            firstPath = PathPlannerPath.fromPathFile("Get Depot Balls From Start");
            secondPath = PathPlannerPath.fromPathFile("Get Depot Balls Shoot Further");

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
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                    drivetrain.getAutoPath(firstPath),
                    drivetrain.getAutoPath(secondPath),
                    new WaitCommand(2.5),
                    new AgitateBalls(intake)

            );

        }

    }

}