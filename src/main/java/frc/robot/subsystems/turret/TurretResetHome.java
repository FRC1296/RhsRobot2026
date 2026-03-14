// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FMJRobotContainer;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class TurretResetHome extends Command {
    private TurretSubsystem turret;
    private DigitalInput turretSensor;
    private boolean done = false;
    private boolean atTarget = false;

    private StatusSignal<Boolean> turretAtTarget;

    /** Creates a new TurretResetHome. */
    public TurretResetHome(FMJRobotContainer robot) {
        this.turret = robot.getTurret();
        this.turretSensor = turret.getTurretSensor();
        turretAtTarget = turret.getTurretAtTarget();

        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        atTarget = false;
        turret.resetStartPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.resetTurretZero();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
