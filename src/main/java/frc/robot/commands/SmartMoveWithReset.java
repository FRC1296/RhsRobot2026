package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LocalizationHelpers;

public class SmartMoveWithReset extends SequentialCommandGroup{
    public SmartMoveWithReset(CommandSwerveDrivetrain drivetrain){
        addCommands(   
            SmartMove.move(drivetrain, 2.6, 4.0, 0.0),
            new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-front", "limelight-rear")),
            SmartMove.move(drivetrain, 2.6, 4.0, 0.0)
        );
    }
    
}