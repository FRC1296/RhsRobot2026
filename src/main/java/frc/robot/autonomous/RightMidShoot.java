package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.AgitateBalls;
import frc.robot.subsystems.intake.IntakeSubsystem;
//import frc.robot.subsystems.vision.Localization;
//import frc.robot.subsystems.vision.LocalizationHelpers;

/** Add your docs here. */
public class RightMidShoot extends AutonomousRoutine{
     public RightMidShoot(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();

        PathPlannerPath firstPath = null;
        PathPlannerPath secondPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Right Mid Shoot");
            secondPath = PathPlannerPath.fromPathFile("Right Mid Shoot Second");
    
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
                    //new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")),
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                    //new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    drivetrain.getAutoPath(firstPath),
                    new AgitateBalls(intake).withTimeout(4),
                    drivetrain.getAutoPath(secondPath),
                    new AgitateBalls(intake)
            );
                    
        }

    }

}