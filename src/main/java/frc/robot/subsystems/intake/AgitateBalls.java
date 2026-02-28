package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AgitateBalls extends SequentialCommandGroup {
      // TODO: TUNE THIS — find the actual encoder value for 45°
    // Your deployed = 0.68, undeployed = 0.1
    // Midpoint is roughly 0.39, adjust based on testing
    private static final double UNSTICK_POSITION = 0.39;

    // Reverse roller speed (negative = reverse)
    private static final double REVERSE_SPEED = -0.50;

    // Wait for deploy motor to reach position (tune based on actual speed)
    private static final double DEPLOY_WAIT = 0.75;

    // How long to run rollers in reverse
    private static final double REVERSE_ROLLER_DURATION = 0.5;

    // Total command timeout (safety)
    private static final double COMMAND_TIMEOUT = 2.0;

  /** Creates a new Agitator. */
  public AgitateBalls(IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
         // Step 1: Stop rollers first so we don't jam harder
            new InstantCommand(intake::stopIntake, intake),

            // Step 2: Move to ~45° (mid) position
            new InstantCommand(intake::moveIntakeToAgitate, intake),

            // Step 3: Wait for arm to physically reach the position
            new WaitCommand(DEPLOY_WAIT),

            // Step 4: Reverse rollers to flip ball inward
            new InstantCommand(() -> intake.runIntakeReverse(REVERSE_SPEED), intake),

            // Step 5: Let them spin in reverse briefly
            new WaitCommand(REVERSE_ROLLER_DURATION),

            // Step 6: Stop rollers and return to deployed position
            new InstantCommand(intake::stopIntake, intake),
            new InstantCommand(intake::deployIntake, intake)
    );
  }
}