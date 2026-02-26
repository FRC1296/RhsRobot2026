package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FMJRobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AutoAimAndShootMoving extends Command {
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private double targetX;
    private double targetY;

    public AutoAimAndShootMoving(FMJRobotContainer robot, double targetX, double targetY) {
        this.shooter = robot.getShooter();
        this.turret = robot.getTurret();
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(shooter, turret);
    }

    public void initialize() {
        turret.turretAimToFeedBool(false);
        turret.turretAimAtHubBool(true);
    }

    @Override
    public void execute() {
        Translation2d virtualTarget = shooter.getVirtualTarget(targetX, targetY);
        turret.turretAimAt(virtualTarget);
        shooter.setAutoShooter(targetX, targetY);
    }

    @Override
    public boolean isFinished() {
        return Constants.turretConstants.turretAimAtHub == false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMasterShooter();
    }
}