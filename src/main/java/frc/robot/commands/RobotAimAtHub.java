package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FMJRobotContainer;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotAimAtHub extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private TurretSubsystem turret;
    private Command move;
    private double targetX;
    private double targetY;
    private Timer timer;

    public RobotAimAtHub(FMJRobotContainer robot, double targetX, double targetY) {
        this.drivetrain = robot.getDrivetrain();
        this.turret = robot.getTurret();
        this.targetX = targetX;
        this.targetY = targetY;
        timer = new Timer();
        addRequirements(drivetrain, turret);
    }

    @Override
    public void initialize() {
        timer.restart();
        turret.turretAimAtHubBool(false);
        turret.moveTurretToZero();
        double targetAngle = drivetrain.aimRobotAt(targetX, targetY);
        move = SmartMove.move(drivetrain, drivetrain.getPose().getX(), drivetrain.getPose().getY(), targetAngle);
        move.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean result = false;
        if(timer.get() > .25){
            result = true;
        }
        return result;
    }

    @Override
    public void end(boolean interrupted) {
    }
}