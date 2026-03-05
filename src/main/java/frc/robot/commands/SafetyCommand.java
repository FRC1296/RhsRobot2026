package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FMJRobotContainer;
import frc.robot.Constants.turretConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SafetyCommand extends ParallelCommandGroup {

    public SafetyCommand(FMJRobotContainer robot) {
        TurretSubsystem turret = robot.getTurret();
        ShooterSubsystem shooter = robot.getShooter();
        addCommands(
                Commands.runOnce(shooter::moveHoodZero, shooter),
                Commands.runOnce(turret::resetTurretZero,turret),
                Commands.runOnce(shooter::setShooterSafeVelocity,shooter)
        );
    }
}
