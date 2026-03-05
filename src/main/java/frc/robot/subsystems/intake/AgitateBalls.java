package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AgitateBalls extends SequentialCommandGroup {

  public AgitateBalls(IntakeSubsystem intake) {

    addCommands(
        new InstantCommand(intake::moveIntakeToAgitate),
        new WaitCommand(1.5),
        new InstantCommand(intake::deployIntake),
        new WaitCommand(1),
        new InstantCommand(intake::moveIntakeToAgitate),
        new WaitCommand(1.5),
        new InstantCommand(intake::deployIntake)
    );
  }
}