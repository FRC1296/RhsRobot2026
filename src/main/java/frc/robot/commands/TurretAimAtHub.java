package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.FMJRobotContainer;

public class TurretAimAtHub extends Command {
    private TurretSubsystem turret;
    private double targetX;
    private double targetY;

    public TurretAimAtHub(FMJRobotContainer robot, double targetX, double targetY) {
        this.turret = robot.getTurret();
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(turret);
    }

    @Override
    public void initialize(){
        turret.turretAimAtHubBool(true);
    }

    @Override
    public void execute() {
        turret.turretAimAt(targetX, targetY);
    }

    @Override
    public boolean isFinished() {
        return Constants.turretConstants.turretAimAtHub == false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}