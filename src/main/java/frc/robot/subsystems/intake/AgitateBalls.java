package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AgitateBalls extends SequentialCommandGroup {

  public AgitateBalls(IntakeSubsystem intake) {

    addCommands(
            new InstantCommand(intake::undeployIntake),
            new WaitCommand(1.0),
            new InstantCommand(intake::stopDeployIntake),
            new WaitCommand(0.5),
            new InstantCommand(intake::setDeployCoast),
            new InstantCommand(intake::deployIntake),
            new WaitCommand(0.5),
            new InstantCommand(intake::stopDeployIntake),
            new InstantCommand(intake::setDeployBrake)
    );
  }
}