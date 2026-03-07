package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootAuton extends AutonomousRoutine {
    public ShootAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        ShooterSubsystem shooter = robot.getShooter();
        FeederSubsystem feeder = robot.getFeeder();
        TurretSubsystem turret = robot.getTurret();
        SpindexerSubsystem spindexer = robot.getSpindexer();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Move Back And Rotate");
        } catch (Exception e) {
            System.err.println("Unable to load PathPlanner file - " + e.getLocalizedMessage());
            pathLoaded = false;
        }

        if (pathLoaded) {
            if (isRedAlliance) {
                firstPath = firstPath.flipPath();
            }

            this.initialPose = firstPath.getStartingHolonomicPose().get();

            addCommands(
                drivetrain.getAutoPath(firstPath),
                    new InstantCommand(spindexer::runSpindexer, spindexer),
                    new InstantCommand(feeder::runFeeder, feeder),
                    new WaitCommand(8),
                    new ParallelCommandGroup(
                            new InstantCommand(spindexer::stopSpindexer),
                            new InstantCommand(feeder::stopFeeder)),
                            new InstantCommand(intake::deployIntake) )
;
        }
    }
}