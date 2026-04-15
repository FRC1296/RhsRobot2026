package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FMJRobotContainer;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootBalls extends ParallelCommandGroup {

    public ShootBalls(FMJRobotContainer robot) {
        ShooterSubsystem shooter = robot.getShooter();
        FeederSubsystem feeder = robot.getFeeder();

        addCommands(
                Commands.startEnd(shooter::runMasterShooter, shooter::stopMasterShooter, shooter)
                //Commands.startEnd(feeder::runFeeder, feeder::stopFeeder, feeder)
        );
    }
}
