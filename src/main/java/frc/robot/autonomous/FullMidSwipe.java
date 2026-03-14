// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/** Add your docs here. */
public class FullMidSwipe extends AutonomousRoutine{
     public FullMidSwipe(FMJRobotContainer robot, double velocity, double acceleration,
            boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        IntakeSubsystem intake = robot.getIntake();
        SpindexerSubsystem spindexer = robot.getSpindexer();
        FeederSubsystem feeder = robot.getFeeder();

        PathPlannerPath firstPath = null;
        // PathPlannerPath secondPath = null;
        // PathPlannerPath thirdPath = null;
        // PathPlannerPath fourthPath = null;
        // PathPlannerPath fifthPath = null;

        boolean pathLoaded = true;
        try {
            firstPath = PathPlannerPath.fromPathFile("Full Mid Swipe");
            // secondPath = PathPlannerPath.fromPathFile("Middle To Intake Pos");
            // thirdPath = PathPlannerPath.fromPathFile("Intake Pos To Midway");
            // fourthPath = PathPlannerPath.fromPathFile("Midway To Depot");
            // fifthPath = PathPlannerPath.fromPathFile("Depot To Shoot Pos");

            
         
        } catch (Exception e) {
            System.err.println("Unable to load PathPlanner file - " + e.getLocalizedMessage());
            pathLoaded = false;
        }

        if (pathLoaded) {
            if (isRedAlliance) {
                firstPath = firstPath.flipPath();
                // secondPath = secondPath.flipPath();
                // thirdPath = thirdPath.flipPath();
                // fourthPath = fourthPath.flipPath();
                // fifthPath = fifthPath.flipPath();
            }

            this.initialPose = firstPath.getStartingHolonomicPose().get();

            addCommands(
                    new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")),
                    new VerifyHeading(robot, initialPose.getRotation().getDegrees()),
                    new InstantCommand(() -> SmartMove.move(drivetrain, initialPose.getX(), initialPose.getY(), 0.0)),
                    new InstantCommand(intake::deployIntake), 
                    new ParallelCommandGroup(
                        new InstantCommand(intake::runIntake),          
                        drivetrain.getAutoPath(firstPath)
                    ),
                    new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")),
                    new ParallelCommandGroup(
                        Commands.runOnce(intake::undeployIntake, intake),
                        new InstantCommand(feeder::runFeeder, feeder),
                        new InstantCommand(spindexer::runSpindexer, spindexer)
                    )

            );
                    
        }

    }

}
