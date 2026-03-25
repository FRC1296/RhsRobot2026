package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AgitateBalls extends SequentialCommandGroup {

  public AgitateBalls(IntakeSubsystem intake) {

    addCommands(
        new InstantCommand(intake::runIntake),
        new InstantCommand(intake::moveIntakeToAgitate),
        new WaitCommand(2.0),
        new InstantCommand(intake::deployIntake),
        new WaitCommand(2.0),
        new InstantCommand(intake::moveIntakeToAgitate),
        new WaitCommand(2.0),
        new InstantCommand(intake::deployIntake),
        new InstantCommand(intake::stopIntake)
    );
  }
}