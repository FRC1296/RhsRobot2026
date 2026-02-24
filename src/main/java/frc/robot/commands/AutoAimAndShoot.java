package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FMJRobotContainer;
import frc.robot.subsystems.spindexer.SpindexerShoot;

public class AutoAimAndShoot extends ParallelCommandGroup {

    public AutoAimAndShoot(FMJRobotContainer robot) {

        TurretAimAtHub autoTurretHub;
        ShooterAutoInterpolate autoInterpolate;
        SpindexerShoot spindexerShoot;

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            autoTurretHub = new TurretAimAtHub(robot, 4.6, 4);
            autoInterpolate = new ShooterAutoInterpolate(robot, 4.6, 4);
        } else {
            autoTurretHub = new TurretAimAtHub(robot, 11.9, 4);
            autoInterpolate = new ShooterAutoInterpolate(robot, 11.9, 4);
        }

        spindexerShoot = new SpindexerShoot(robot);

        addCommands(
                autoInterpolate,
                autoTurretHub);
                //spindexerShoot);
    }

}
