package frc.robot.autonomous.routes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FMJRobotContainer;
import frc.robot.autonomous.IAuto;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AutonomousRoutine extends SequentialCommandGroup implements IAuto {
    protected FMJRobotContainer theRobot;
    protected double maxVelocity;
    protected double maxAcceleration;
    protected boolean redPath;
    protected Pose2d initialPose;

    /** Contructor */
    public AutonomousRoutine(FMJRobotContainer robot, double velocity, double acceleration, boolean isRedAlliance) {
        this.theRobot = robot;
        this.maxVelocity = velocity;
        this.maxAcceleration = acceleration;
        this.redPath = isRedAlliance;
    }

    /**
     * Constructor with multiple paths to drive
     * @param robot
     * @param velocity
     * @param acceleration
     * @param driveCommands
     */
    public AutonomousRoutine(FMJRobotContainer robot, double velocity, double acceleration, Command... driveCommands) {
        this.theRobot = robot;
        this.maxVelocity = velocity;
        this.maxAcceleration = acceleration;

        CommandSwerveDrivetrain drivetrain = robot.getDrivetrain();
        this.addRequirements(drivetrain);
        
        if (driveCommands != null && driveCommands.length > 0) {
            for(int i=0; i < driveCommands.length; i++) {
                addCommands(driveCommands[i]);
            }
        }
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(Commands.print("Auton Complete."));
    }

    /**
     * This method will return the initial pose for our robot. It will most likely come from the
     * trajectory information.
     * 
     * @return Pose2d representation of initial position of our robot
     */
    public Pose2d getInitialPose() {
        return initialPose;
    }
}
