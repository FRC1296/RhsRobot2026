package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.routes.AutonomousRoutine;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
//import frc.robot.subsystems.vision.Localization;
//import frc.robot.subsystems.vision.LocalizationHelpers;

/** Add your docs here. */
public class DefaultAuton extends AutonomousRoutine{
     public DefaultAuton(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();

        PathPlannerPath firstPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Default Auton");
            
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
                    //new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")),
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
                    new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    drivetrain.getAutoPath(firstPath)
            );
                    
        }

    }

}