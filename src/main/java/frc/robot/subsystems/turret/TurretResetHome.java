// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

    /** Creates a new TurretResetHome. */
    public TurretResetHome(FMJRobotContainer robot) {
        this.turret = robot.getTurret();
        this.turretSensor = robot.getTurretSensor();

        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        turret.setTurretAngle(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (turretSensor.get()){
            turret.runTurret();
        } else {
            turret.stopTurret();
            done = true;
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
