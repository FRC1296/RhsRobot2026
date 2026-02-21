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
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootAuton extends AutonomousRoutine {
    public ShootAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        // IntakeSubsystem intake = robot.getIntake();
        ShooterSubsystem shooter = robot.getShooter();
        FeederSubsystem feeder = robot.getFeeder();
        TurretSubsystem turret = robot.getTurret();
        SpindexerSubsystem spindexer = robot.getSpindexer();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;

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

                    new AutoAimAndShoot(robot),
                    new InstantCommand(spindexer::runSpindexer, spindexer),
                    new WaitCommand(3),
                    new ParallelCommandGroup(
                            new InstantCommand(spindexer::stopSpindexer),
                            new InstantCommand(shooter::stopAutoAimAndShoot)));
        }
    }
}