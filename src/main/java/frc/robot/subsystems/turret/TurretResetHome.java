package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FMJRobotContainer;

public class TurretResetHome extends Command {
    private TurretSubsystem turret;
    private DigitalInput turretSensor;
    private boolean done = false;
    private boolean atTarget = false;

    private StatusSignal<Boolean> turretAtTarget;

    public TurretResetHome(FMJRobotContainer robot) {
        this.turret = robot.getTurret();
        this.turretSensor = turret.getTurretSensor();
        turretAtTarget = turret.getTurretAtTarget();

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        done = false;
        atTarget = false;
        turret.resetStartPosition();
    }

    @Override
    public void execute() {
        BaseStatusSignal.refreshAll(turretAtTarget);
        if (turretAtTarget.getValue()) {
            atTarget = true;
        }
        if (atTarget){
            if (turretSensor.get()) {
                turret.reverseTurret();
            } else {
                turret.stopTurret();
                done = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.resetTurretZero();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
