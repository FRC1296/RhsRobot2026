package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FMJRobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterAutoInterpolate extends Command{
    private ShooterSubsystem shooter;
    private double targetX;
    private double targetY;

    public ShooterAutoInterpolate(FMJRobotContainer robot, double targetX, double targetY) {
        this.shooter = robot.getShooter();
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Constants.shooterConstants.shooterInterpolate = true;
    }

    @Override
    public void execute() {
        shooter.setAutoShooter(targetX, targetY);
    }

    @Override
    public boolean isFinished() {
        return Constants.shooterConstants.shooterInterpolate == false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
