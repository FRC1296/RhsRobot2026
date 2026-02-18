package frc.robot.autonomous;

import java.util.List;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LocalizationHelpers;

public class DestoryerHistoryMakingTestAuton extends AutonomousRoutine {
    public DestoryerHistoryMakingTestAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        TurretSubsystem turret = robot.getTurret();
        ShooterSubsystem shooter = robot.getShooter();
        ClimberSubsystem climber = robot.getClimber();
        FeederSubsystem feeder = robot.getFeeder();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;
        PathPlannerPath thirdPath = null;
        
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
                            new InstantCommand(intake::deployIntake),
                            new AutoAimAndShoot(robot)),
                    new WaitCommand(1),
                    new InstantCommand(feeder::runSpindexer),
                    new WaitCommand(3),
                    new ParallelCommandGroup(
                            new InstantCommand(feeder::stopSpindexer),
                            new InstantCommand(shooter::stopShooterAutoInterpolate)),
                     drivetrain.getAutoPath(secondPath),
                     new InstantCommand(intake::runIntake),
                     new WaitCommand(3),
                     new AutoAimAndShoot(robot),
                     new InstantCommand(feeder::runSpindexer),
                     new WaitCommand(3),
                      new ParallelCommandGroup(
                            new InstantCommand(feeder::stopSpindexer),
                            new InstantCommand(shooter::stopShooterAutoInterpolate))

            // new InstantCommand(intake::deployIntake),

            // // new InstantCommand(intake::undeployIntake),
            // new InstantCommand(elevator::moveToL4Coral)),
            // new WaitCommand(0.5),
            // new ReefAlignDriveCommand(robot, true).withTimeout(2),
            // new InstantCommand(arm::moveToScoreLeft),
            // new WaitCommand(0.9),
            // new ParallelCommandGroup(
            // new InstantCommand(arm::expelCoral),
            // drivetrain.getAutoPath(secondPath)),
            // // new InstantCommand(intake::deployIntake),
            // new InstantCommand(arm::stopHandMotor),
            // new InstantCommand(arm::moveTo180Degree),
            // new ParallelCommandGroup(
            // drivetrain.getAutoPath(thirdPath),
            // new InstantCommand(intake::runIntake),
            // new InstantCommand(elevator::moveToL3Coral),
            // new InstantCommand(arm::moveToFloorIntakePosition)),
            // new InstantCommand(intake::runIntake).withTimeout(10)
            // new InstantCommand(elevator::moveToL2nAHalfCoral),
            // new WaitCommand(0.8),
            // new InstantCommand(intake::stopIntake),
            // new ParallelCommandGroup(
            // drivetrain.getAutoPath(fourthPath)
            // ),
            // new ArmCoralFromGround(robot),
            // new WaitCommand(1.5),
            // new ParallelCommandGroup(
            // new InstantCommand(elevator::moveToL4Coral),
            // new InstantCommand(arm::moveToReadyPosition)
            // ),
            // new WaitCommand(0.7),
            // new ReefAlignDriveCommand(robot, true).withTimeout(2),
            // new InstantCommand(arm::moveToScoreLeft),
            // new WaitCommand(0.8),
            // new ParallelCommandGroup(
            // new InstantCommand(arm::expelCoral),
            // drivetrain.getAutoPath(fifthPath)
            );

        }

    }
}