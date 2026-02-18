package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FMJRobotContainer;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class VerifyHeading extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private double initialHeading = 0.0;
    private double currHeading = 0.0;

    /** Creates a new VerifyHeading. */
    public VerifyHeading(FMJRobotContainer robot, double pathHeading) {
        drivetrain = robot.getDrivetrain();
        initialHeading = pathHeading;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currHeading = drivetrain.getPose().getRotation().getDegrees();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
     public boolean isFinished() {
      boolean result = false;
    
      if (currHeading >= initialHeading - 1 && currHeading <= initialHeading + 1 ) {
       result = true;
      }
    
      return result;
     }
}
