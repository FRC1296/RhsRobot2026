package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.FMJRobotContainer;

public class TurretAimToFeed extends Command {
    private TurretSubsystem turret;
    private double targetX;
    private double targetY;

    public TurretAimToFeed(FMJRobotContainer robot) {
        this(robot, 0.0, 0.0);
    }

    public TurretAimToFeed(FMJRobotContainer robot, double targetX, double targetY) {
        this.turret = robot.getTurret();
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(turret);
    }

    @Override
    public void initialize(){
        turret.turretAimAtHubBool(false);
        turret.turretAimToFeedBool(true);
    }

    @Override
    public void execute() {
        //turret.turretAimAt(targetX, targetY);
    }

    @Override
    public boolean isFinished() {
        return Constants.turretConstants.turretAimToFeed == false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public void setFeedLocation(Translation2d location) {
        targetX = location.getX();
        targetY = location.getY();
    }
}